"""Support for Onkyo Receivers."""
from __future__ import annotations

import logging
import time
from typing import Any

import eiscp
from eiscp import eISCP
import voluptuous as vol

from homeassistant.components.media_player import (
    DOMAIN,
    PLATFORM_SCHEMA,
    MediaPlayerEntity,
    MediaPlayerEntityFeature,
    MediaPlayerState,
    MediaType,
)
from homeassistant.const import ATTR_ENTITY_ID, CONF_HOST, CONF_NAME
from homeassistant.core import HomeAssistant, ServiceCall
import homeassistant.helpers.config_validation as cv
from homeassistant.helpers.entity_platform import AddEntitiesCallback
from homeassistant.helpers.typing import ConfigType, DiscoveryInfoType

_LOGGER = logging.getLogger(__name__)

CONF_SOURCES = "sources"
# separate config entry for network sources (apps)
# to allow for selecting network apps as sources via GUI
CONF_NET_SOURCES = "net_sources"
CONF_MAX_VOLUME = "max_volume"
CONF_RECEIVER_MAX_VOLUME = "receiver_max_volume"
CONF_SOUND_MODES = "sounc_modes"

DEFAULT_NAME = "Onkyo Receiver"
SUPPORTED_MAX_VOLUME = 100
DEFAULT_RECEIVER_MAX_VOLUME = 80


SUPPORT_ONKYO_WO_VOLUME = (
    MediaPlayerEntityFeature.TURN_ON
    | MediaPlayerEntityFeature.TURN_OFF
    | MediaPlayerEntityFeature.SELECT_SOURCE
    | MediaPlayerEntityFeature.PLAY
    | MediaPlayerEntityFeature.PLAY_MEDIA
    | MediaPlayerEntityFeature.SELECT_SOUND_MODE
)
SUPPORT_ONKYO = (
    SUPPORT_ONKYO_WO_VOLUME
    | MediaPlayerEntityFeature.VOLUME_SET
    | MediaPlayerEntityFeature.VOLUME_MUTE
    | MediaPlayerEntityFeature.VOLUME_STEP
)

KNOWN_HOSTS: list[str] = []
# default list of sources
# <internalName> : <HA Name>
DEFAULT_SOURCES = {
    "tv": "TV",
    "bd": "Bluray",
    "game": "Game",
    "aux1": "Aux1",
    "video1": "Video 1",
    "video2": "Video 2",
    "video3": "Video 3",
    "video4": "Video 4",
    "video5": "Video 5",
    "video6": "Video 6",
    "video7": "Video 7",
    "fm": "Radio",
    "net": "Network",
}


# Default Net sources, key is the hex number of the source, value the text
# appearing in HA User INterface
# https://github.com/miracle2k/onkyo-eiscp/blob/master/eiscp-commands.yaml
# command NLT
DEFAULT_NET_SOURCES = {
    "00": "DLNA",
    # "01" : "Favorite",
    # "02" : "vTuner",
    # "03" : "SiriusXM",
    "04": "Pandora",
    # "05" : "Rhapsody",
    # "06" : "Last.fm",
    # "07" : "Napster",
    # "08" : "Slacker",
    # "09" : "Mediafly",
    "0A": "Spotify",
    # "0B" : "AUPEO!",
    # "0C" : "radiko",
    # "0D" : "e-onkyo",
    "0E": "TuneIn Radio",
    # 0F" : "MP3tunes",
    # "10" : "Simfy",
    # "11" : "Home Media",
    "12": "Deezer",
    # "13" : "iHeartRadio",
    # manually
    "40": "Chromecast",
    # additional from excel
    # https://michael.elsdoerfer.name/onkyo/ISCP_AVR_2014.Models.xlsx
    "F0": "USB Front",
    "F1": "USB Rear",
    # "F2" : "Internet Radio",
    "F3": "NET-Menu",
    "FF": "None",
}


DEFAULT_SOUND_MODES = {
    "all": "All",
    "FR": "Front",
    "SR": "Dinner",  # Surround
    "H1": "Kitchen",  # Height 1
}

DEFAULT_PLAYABLE_SOURCES = ("fm", "am", "tuner")

PLATFORM_SCHEMA = PLATFORM_SCHEMA.extend(
    {
        vol.Optional(CONF_HOST): cv.string,
        vol.Optional(CONF_NAME, default=DEFAULT_NAME): cv.string,
        vol.Optional(CONF_MAX_VOLUME, default=SUPPORTED_MAX_VOLUME): vol.All(
            vol.Coerce(int), vol.Range(min=1, max=100)
        ),
        vol.Optional(
            CONF_RECEIVER_MAX_VOLUME, default=DEFAULT_RECEIVER_MAX_VOLUME
        ): cv.positive_int,
        vol.Optional(CONF_SOURCES, default=DEFAULT_SOURCES): {cv.string: cv.string},
        vol.Optional(CONF_NET_SOURCES, default=DEFAULT_NET_SOURCES): {
            cv.string: cv.string
        },
        vol.Optional(CONF_SOUND_MODES, default=DEFAULT_SOUND_MODES): {
            cv.string: cv.string
        },
    }
)

TIMEOUT_MESSAGE = "Timeout waiting for response."


ATTR_HDMI_OUTPUT = "hdmi_output"
ATTR_PRESET = "preset"
ATTR_AUDIO_INFORMATION = "audio_information"
ATTR_VIDEO_INFORMATION = "video_information"
ATTR_VIDEO_OUT = "video_out"

ACCEPTED_VALUES = [
    "no",
    "analog",
    "yes",
    "out",
    "out-sub",
    "sub",
    "hdbaset",
    "both",
    "up",
]
ONKYO_SELECT_OUTPUT_SCHEMA = vol.Schema(
    {
        vol.Required(ATTR_ENTITY_ID): cv.entity_ids,
        vol.Required(ATTR_HDMI_OUTPUT): vol.In(ACCEPTED_VALUES),
    }
)

SERVICE_SELECT_HDMI_OUTPUT = "onkyo_select_hdmi_output"

# only allow 1 parallel call for this integration as the
# underlying library is not thread safe
# otherwise we get concurrent calls.
# NOTE although restricted, it seems we are still getting parallel calls
#  between an update and set function (e.g. select_input, volume_down,
# and especially the longer running service calls for playing media)
# Debugging has shown that the parallel_updates semaphore is locked
# when the call arrives in the integration (but I don't see the )
PARALLEL_UPDATES = 1


def _parse_onkyo_payload(payload):
    """Parse a payload returned from the eiscp library."""
    if isinstance(payload, bool):
        # command not supported by the device
        return False

    if len(payload) < 2:
        # no value
        return None

    if isinstance(payload[1], str):
        return payload[1].split(",")

    return payload[1]


def _tuple_get(tup, index, default=None):
    """Return a tuple item at index or a default value if it doesn't exist."""
    return (tup[index : index + 1] or [default])[0]


def determine_zones(receiver):
    """Determine what zones are available for the receiver."""
    out = {"zone2": False, "zone3": False}
    try:
        _LOGGER.debug("Checking for zone 2 capability")
        response = receiver.raw("ZPWQSTN")
        if response != "ZPWN/A":  # Zone 2 Available
            out["zone2"] = True
        else:
            _LOGGER.debug("Zone 2 not available")
    except ValueError as error:
        if str(error) != TIMEOUT_MESSAGE:
            raise error
        _LOGGER.debug("Zone 2 timed out, assuming no functionality")
    try:
        _LOGGER.debug("Checking for zone 3 capability")
        response = receiver.raw("PW3QSTN")
        if response != "PW3N/A":
            out["zone3"] = True
        else:
            _LOGGER.debug("Zone 3 not available")
    except ValueError as error:
        if str(error) != TIMEOUT_MESSAGE:
            raise error
        _LOGGER.debug("Zone 3 timed out, assuming no functionality")
    except AssertionError:
        _LOGGER.error("Zone 3 detection failed")

    return out


def setup_platform(
    hass: HomeAssistant,
    config: ConfigType,
    add_entities: AddEntitiesCallback,
    discovery_info: DiscoveryInfoType | None = None,
) -> None:
    """Set up the Onkyo platform."""
    hosts: list[OnkyoDevice] = []

    def service_handle(service: ServiceCall) -> None:
        """Handle for services."""
        entity_ids = service.data[ATTR_ENTITY_ID]
        devices = [d for d in hosts if d.entity_id in entity_ids]

        for device in devices:
            if service.service == SERVICE_SELECT_HDMI_OUTPUT:
                device.select_output(service.data[ATTR_HDMI_OUTPUT])

    hass.services.register(
        DOMAIN,
        SERVICE_SELECT_HDMI_OUTPUT,
        service_handle,
        schema=ONKYO_SELECT_OUTPUT_SCHEMA,
    )

    if CONF_HOST in config and (host := config[CONF_HOST]) not in KNOWN_HOSTS:
        try:
            receiver = eiscp.eISCP(host)
            hosts.append(
                OnkyoDevice(
                    receiver,
                    config.get(CONF_SOURCES),
                    config.get(CONF_NET_SOURCES),
                    config.get(CONF_SOUND_MODES),
                    name=config.get(CONF_NAME),
                    max_volume=config.get(CONF_MAX_VOLUME),
                    receiver_max_volume=config.get(CONF_RECEIVER_MAX_VOLUME),
                )
            )
            KNOWN_HOSTS.append(host)

            zones = determine_zones(receiver)

            # Add Zone2 if available
            if zones["zone2"]:
                _LOGGER.debug("Setting up zone 2")
                hosts.append(
                    OnkyoDeviceZone(
                        "2",
                        receiver,
                        config.get(CONF_SOURCES),
                        config.get(CONF_NET_SOURCES),
                        config.get(CONF_SOUND_MODES),
                        name=f"{config[CONF_NAME]} Zone 2",
                        max_volume=config.get(CONF_MAX_VOLUME),
                        receiver_max_volume=config.get(CONF_RECEIVER_MAX_VOLUME),
                    )
                )
            # Add Zone3 if available
            if zones["zone3"]:
                _LOGGER.debug("Setting up zone 3")
                hosts.append(
                    OnkyoDeviceZone(
                        "3",
                        receiver,
                        config.get(CONF_SOURCES),
                        config.get(CONF_NET_SOURCES),
                        config.get(CONF_SOUND_MODES),
                        name=f"{config[CONF_NAME]} Zone 3",
                        max_volume=config.get(CONF_MAX_VOLUME),
                        receiver_max_volume=config.get(CONF_RECEIVER_MAX_VOLUME),
                    )
                )
        except OSError:
            _LOGGER.error("Unable to connect to receiver at %s", host)
    else:
        for receiver in eISCP.discover():
            if receiver.host not in KNOWN_HOSTS:
                hosts.append(
                    OnkyoDevice(
                        receiver,
                        config.get(CONF_SOURCES),
                        config.get(CONF_NET_SOURCES),
                        config.get(CONF_SOUND_MODES),
                    )
                )
                KNOWN_HOSTS.append(receiver.host)
    add_entities(hosts, True)


class OnkyoDevice(MediaPlayerEntity):
    """Representation of an Onkyo device."""

    _attr_supported_features = SUPPORT_ONKYO

    def __init__(
        self,
        receiver,
        sources,
        net_sources,
        sound_modes,
        name=None,
        max_volume=SUPPORTED_MAX_VOLUME,
        receiver_max_volume=DEFAULT_RECEIVER_MAX_VOLUME,
    ) -> None:
        """Initialize the Onkyo Receiver."""
        self._receiver = receiver
        self._attr_is_volume_muted = False
        self._attr_volume_level = 0
        self._attr_state = MediaPlayerState.OFF
        if name:
            # not discovered
            self._attr_name = name
        else:
            # discovered
            self._attr_unique_id = (
                f"{receiver.info['model_name']}_{receiver.info['identifier']}"
            )
            self._attr_name = self._attr_unique_id

        self._max_volume = max_volume
        self._receiver_max_volume = receiver_max_volume
        #  show combined sources and net sources in pull down list
        self._attr_source_list = list(sources.values()) + list(net_sources.values())
        self._source_mapping = sources
        self._source_net_mapping = net_sources
        # combine the dynamic (user defined source list)
        # and the static NET sources list so we can select
        # TODO, unsure why reverse mapping needs to be given too (not _attr)
        combined = {**sources, **net_sources}
        self._reverse_mapping = {value: key for key, value in combined.items()}
        # separate for lookup if this is a net source
        self._reverse_mapping_net = {value: key for key, value in net_sources.items()}
        self._attr_extra_state_attributes = {}
        self._hdmi_out_supported = True
        self._audio_info_supported = True
        self._video_info_supported = True
        self.call_nr = 0  # number of concurrent calls ... yes it is not thread safe
        self._sound_modes = sound_modes
        self._sound_modes_reverse = {value: key for key, value in sound_modes.items()}
        self._attr_sound_mode_list = list(sound_modes.values())
        self._attr_sound_mode = self._sound_modes["all"]

    def command(self, command):
        """Run an eiscp command and catch connection errors."""
        try:
            result = self._receiver.command(command)
        except (ValueError, OSError, AttributeError, AssertionError) as my_ex:
            if self._receiver.command_socket:
                self._receiver.command_socket = None
                _LOGGER.debug("Sending %s resulted in exception:", command)
                _LOGGER.debug(str(my_ex))
                _LOGGER.debug("Resetting connection to %s", self.name)
            else:
                _LOGGER.info("%s is disconnected. Attempting to reconnect", self.name)
            return False
        _LOGGER.debug("Result for %s: %s", command, result)
        return result

    def raw(self, command):
        """Run an eiscp raw command and catch connection errors."""
        try:
            result = self._receiver.raw(command)
        except (ValueError, OSError, AttributeError, AssertionError) as my_ex:
            if self._receiver.command_socket:
                self._receiver.command_socket = None
                _LOGGER.debug("Sending %s resulted in exception:", command)
                _LOGGER.debug(my_ex)
                _LOGGER.debug("Resetting connection to %s", self.name)
            else:
                _LOGGER.info("%s is disconnected. Attempting to reconnect", self.name)
            return False
        _LOGGER.debug("Result for %s: %s", command, result)
        return result

    def send(self, command):
        """Run a raw eiscp send without any confirmation check but  catch connection errors."""
        try:
            self._receiver.send(command)
        except (ValueError, OSError, AttributeError, AssertionError) as my_ex:
            if self._receiver.command_socket:
                self._receiver.command_socket = None
                _LOGGER.debug("Sending %s resulted in exception:", command)
                _LOGGER.debug(my_ex)
                _LOGGER.debug("Resetting connection to %s", self.name)
            else:
                _LOGGER.info("%s is disconnected. Attempting to reconnect", self.name)
            return False
        _LOGGER.debug("Sent cmd (no resp. check) %s", command)

    def filter_for_message(self, command):
        """Wait until a message that starts with command is received or timeout."""
        try:
            start = time.time()
            while True:
                candidate = self._receiver.get(0.05)
                # It seems ISCP commands are always three characters.
                if candidate and candidate.startswith(command):
                    _LOGGER.debug("Awaited result for %s:  %s", command, candidate)
                    return candidate
                # The protocol docs claim that a response  should arrive
                # within *50ms or the communication has failed*. In my tests,
                # however, the interval needed to be at least 200ms before
                # I managed to see any response, and only after 300ms
                # on a regular basis, so use a generous timeout.
                if time.time() - start > 5.0:
                    raise ValueError("Timeout waiting for response.")

        except (ValueError, OSError, AttributeError, AssertionError) as my_ex:
            if self._receiver.command_socket:
                self._receiver.command_socket = None
                _LOGGER.debug("Filter for %s resulted in exception:", command)
                _LOGGER.debug(my_ex)
                _LOGGER.debug("Resetting connection to %s", self.name)
            else:
                _LOGGER.info("%s is disconnected. Attempting to reconnect", self.name)
            return False

    def update(self) -> None:
        """Get the latest state from the device."""
        status = self.command("system-power query")

        if not status:
            return
        if status[1] == "on":
            self._attr_state = MediaPlayerState.ON
        else:
            self._attr_state = MediaPlayerState.OFF
            self._attr_extra_state_attributes.pop(ATTR_AUDIO_INFORMATION, None)
            self._attr_extra_state_attributes.pop(ATTR_VIDEO_INFORMATION, None)
            self._attr_extra_state_attributes.pop(ATTR_PRESET, None)
            self._attr_extra_state_attributes.pop(ATTR_VIDEO_OUT, None)
            return

        volume_raw = self.command("volume query")
        mute_raw = self.command("audio-muting query")
        current_source_raw = self.command("input-selector query")
        # If the following command is sent to a device with only one HDMI out,
        # the display shows 'Not Available'.
        # We avoid this by checking if HDMI out is supported
        if self._hdmi_out_supported:
            hdmi_out_raw = self.command("hdmi-output-selector query")
        else:
            hdmi_out_raw = []
        preset_raw = self.command("preset query")
        if self._audio_info_supported:
            audio_information_raw = self.command("audio-information query")
            self._parse_audio_information(audio_information_raw)
        if self._video_info_supported:
            video_information_raw = self.command("video-information query")
            self._parse_video_information(video_information_raw)
        if not (volume_raw and mute_raw and current_source_raw):
            return

        sources = _parse_onkyo_payload(current_source_raw)

        # Net as a source selector branches out to many apps,
        # check which app is running
        if "net" in sources:
            net_stat = self.raw("NLTQSTN")
            # extract source ID (response contains much more)
            #  if we got one
            if net_stat:
                net_src_id = net_stat[3:5]
                # if ID exists, set the discoverd sources
                if self._source_net_mapping.get(net_src_id):
                    self._attr_source = self._source_net_mapping[net_src_id]
                    sources = (
                        []
                    )  # set to empty to avoid lookup of regular sources (next)

        for source in sources:
            if source in self._source_mapping:
                self._attr_source = self._source_mapping[source]
                break
            self._attr_source = "_".join(sources)

        if preset_raw and self.source and self.source.lower() == "radio":
            self._attr_extra_state_attributes[ATTR_PRESET] = preset_raw[1]
        elif ATTR_PRESET in self._attr_extra_state_attributes:
            del self._attr_extra_state_attributes[ATTR_PRESET]

        self._attr_is_volume_muted = bool(mute_raw[1] == "on")
        #       AMP_VOL/MAX_RECEIVER_VOL*(MAX_VOL/100)
        self._attr_volume_level = volume_raw[1] / (
            self._receiver_max_volume * self._max_volume / 100
        )

        if not hdmi_out_raw:
            return
        self._attr_extra_state_attributes[ATTR_VIDEO_OUT] = ",".join(hdmi_out_raw[1])
        if hdmi_out_raw[1] == "N/A":
            self._hdmi_out_supported = False

        # Don't change sound mode in TV, list as all
        if self._attr_source and self._attr_source.lower() == "tv":
            self._attr_sound_mode = self._sound_modes_reverse["all"]
        else:
            # query current mode
            current_listenmode_raw = self.command("listening-mode query")
            current_listenmode = _parse_onkyo_payload(current_listenmode_raw)
            if current_listenmode[0] != "stereo":
                self._attr_sound_mode = self._sound_modes["all"]
            else:
                # need do query current stereo assign value
                sam_current_raw = self.raw("SAMQSTN")
                sam_id = sam_current_raw[3:5]
                _LOGGER.debug("SAM: %s", sam_id)
                if sam_id in self._sound_modes:
                    self._attr_sound_mode = self._sound_modes[sam_id]

    def turn_off(self) -> None:
        """Turn the media player off."""
        self.command("system-power standby")

    def set_volume_level(self, volume: float) -> None:
        """Set volume level, input is range 0..1.

        However full volume on the amp is usually far too loud so allow the user to
        specify the upper range with CONF_MAX_VOLUME. We change as per max_volume
        set by user. This means that if max volume is 80 then full volume in HA will
        give 80% volume on the receiver. Then we convert that to the correct scale
        for the receiver.
        """
        #        HA_VOL * (MAX VOL / 100) * MAX_RECEIVER_VOL
        self.command(
            "volume"
            f" {int(volume * (self._max_volume / 100) * self._receiver_max_volume)}"
        )

    def volume_up(self) -> None:
        """Increase volume by 1 step."""
        self.command("volume level-up")

    def volume_down(self) -> None:
        """Decrease volume by 1 step."""
        self.command("volume level-down")

    def mute_volume(self, mute: bool) -> None:
        """Mute (true) or unmute (false) media player."""
        if mute:
            self.command("audio-muting on")
        else:
            self.command("audio-muting off")

    def turn_on(self) -> None:
        """Turn the media player on."""
        self.command("system-power on")

    def select_source(self, source: str) -> None:
        """Set the input source."""
        # is the target source a network source?
        if source in self._reverse_mapping_net:
            net_src_id = self._reverse_mapping_net[source]
            # select apps input first, to then select which app to take
            self.command("input-selector net")
            # just send the command without checking, the next update should fetch it
            # trailing 0 indicates no username / password for the receiver app
            self.send(f"NSV{net_src_id}0")

            # wait for the response which is in form of "NLT.*",
            # to pass in the query version to ensure valid command (although unneeded)
            self.filter_for_message(f"NLT{net_src_id}")
        else:
            if self.source_list and source in self.source_list:
                source = self._reverse_mapping[source]
            self.command(f"input-selector {source}")

    def select_sound_mode(self, sound_mode: str) -> None:
        """Select sound mode."""
        # should expand to only change in music mode
        if sound_mode in self._sound_modes_reverse:
            sm_id = self._sound_modes_reverse[sound_mode]
            if sm_id == "all":
                # set to all stereo
                self.command("listening-mode=all-ch-stereo")
            else:
                # sound mode stereo
                # self.command("listening-mode=stereo")
                # use raw command, above did result in N/A
                self.raw("LMD00")
                # "SAMFR" front
                # "SAMSR" surround
                # "SAMH1" Height1
                # set the stereo assign mode
                self.raw(f"SAM{sm_id}")

    def play_media(
        self, media_type: MediaType | str, media_id: str, **kwargs: Any
    ) -> None:
        """Play radio station by preset number."""
        source = self._reverse_mapping[self._attr_source]
        if media_type.lower() == "radio" and source in DEFAULT_PLAYABLE_SOURCES:
            self.command(f"preset {media_id}")
        # play "0E": "TuneIn Radio" preset
        if media_type.lower() == self._source_net_mapping["0E"].lower():
            # force switch to TuneIn Radio since we continue relative from there
            self.select_source(media_type)
            # now we should be in the menu for TuneIn Radio, and see the first option
            self.filter_for_message("NLSU0")
            # first entry is My Presets (NLSU0 actually)
            self.send("NLSI00001")
            # now we should see a list of presets, wait until the first preset is shown
            self.filter_for_message("NLSU0")
            # select the preset by id which should be single didigt
            self.send(f"NLSI0000{media_id}")
            # no checking?> the selection should change the status and we should get something back

    def select_output(self, output):
        """Set hdmi-out."""
        self.command(f"hdmi-output-selector={output}")

    def _parse_audio_information(self, audio_information_raw):
        values = _parse_onkyo_payload(audio_information_raw)
        if values is False:
            self._audio_info_supported = False
            return

        if values:
            info = {
                "format": _tuple_get(values, 1),
                "input_frequency": _tuple_get(values, 2),
                "input_channels": _tuple_get(values, 3),
                "listening_mode": _tuple_get(values, 4),
                "output_channels": _tuple_get(values, 5),
                "output_frequency": _tuple_get(values, 6),
            }
            self._attr_extra_state_attributes[ATTR_AUDIO_INFORMATION] = info
        else:
            self._attr_extra_state_attributes.pop(ATTR_AUDIO_INFORMATION, None)

    def _parse_video_information(self, video_information_raw):
        values = _parse_onkyo_payload(video_information_raw)
        if values is False:
            self._video_info_supported = False
            return

        if values:
            info = {
                "input_resolution": _tuple_get(values, 1),
                "input_color_schema": _tuple_get(values, 2),
                "input_color_depth": _tuple_get(values, 3),
                "output_resolution": _tuple_get(values, 5),
                "output_color_schema": _tuple_get(values, 6),
                "output_color_depth": _tuple_get(values, 7),
                "picture_mode": _tuple_get(values, 8),
            }
            self._attr_extra_state_attributes[ATTR_VIDEO_INFORMATION] = info
        else:
            self._attr_extra_state_attributes.pop(ATTR_VIDEO_INFORMATION, None)


class OnkyoDeviceZone(OnkyoDevice):
    """Representation of an Onkyo device's extra zone."""

    def __init__(
        self,
        zone,
        receiver,
        sources,
        net_sources,
        sound_modes,
        name=None,
        max_volume=SUPPORTED_MAX_VOLUME,
        receiver_max_volume=DEFAULT_RECEIVER_MAX_VOLUME,
    ) -> None:
        """Initialize the Zone with the zone identifier."""
        self._zone = zone
        self._supports_volume = True
        super().__init__(
            receiver,
            sources,
            net_sources,
            sound_modes,
            name,
            max_volume,
            receiver_max_volume,
        )

    def update(self) -> None:
        """Get the latest state from the device."""
        status = self.command(f"zone{self._zone}.power=query")

        if not status:
            return
        if status[1] == "on":
            self._attr_state = MediaPlayerState.ON
        else:
            self._attr_state = MediaPlayerState.OFF
            return

        volume_raw = self.command(f"zone{self._zone}.volume=query")
        mute_raw = self.command(f"zone{self._zone}.muting=query")
        current_source_raw = self.command(f"zone{self._zone}.selector=query")
        preset_raw = self.command(f"zone{self._zone}.preset=query")
        # If we received a source value, but not a volume value
        # it's likely this zone permanently does not support volume.
        if current_source_raw and not volume_raw:
            self._supports_volume = False

        if not (volume_raw and mute_raw and current_source_raw):
            return

        # It's possible for some players to have zones set to HDMI with
        # no sound control. In this case, the string `N/A` is returned.
        self._supports_volume = isinstance(volume_raw[1], (float, int))

        # eiscp can return string or tuple. Make everything tuples.
        if isinstance(current_source_raw[1], str):
            current_source_tuples = (current_source_raw[0], (current_source_raw[1],))
        else:
            current_source_tuples = current_source_raw

        for source in current_source_tuples[1]:
            if source in self._source_mapping:
                self._attr_source = self._source_mapping[source]
                break
            self._attr_source = "_".join(current_source_tuples[1])
        self._attr_is_volume_muted = bool(mute_raw[1] == "on")
        if preset_raw and self.source and self.source.lower() == "radio":
            self._attr_extra_state_attributes[ATTR_PRESET] = preset_raw[1]
        elif ATTR_PRESET in self._attr_extra_state_attributes:
            del self._attr_extra_state_attributes[ATTR_PRESET]
        if self._supports_volume:
            # AMP_VOL/MAX_RECEIVER_VOL*(MAX_VOL/100)
            self._attr_volume_level = (
                volume_raw[1] / self._receiver_max_volume * (self._max_volume / 100)
            )

    @property
    def supported_features(self) -> MediaPlayerEntityFeature:
        """Return media player features that are supported."""
        if self._supports_volume:
            return SUPPORT_ONKYO
        return SUPPORT_ONKYO_WO_VOLUME

    def turn_off(self) -> None:
        """Turn the media player off."""
        self.command(f"zone{self._zone}.power=standby")

    def set_volume_level(self, volume: float) -> None:
        """Set volume level, input is range 0..1.

        However full volume on the amp is usually far too loud so allow the user to
        specify the upper range with CONF_MAX_VOLUME. We change as per max_volume
        set by user. This means that if max volume is 80 then full volume in HA
        will give 80% volume on the receiver. Then we convert that to the correct
        scale for the receiver.
        """
        # HA_VOL * (MAX VOL / 100) * MAX_RECEIVER_VOL
        self.command(
            f"zone{self._zone}.volume={int(volume * (self._max_volume / 100) * self._receiver_max_volume)}"
        )

    def volume_up(self) -> None:
        """Increase volume by 1 step."""
        self.command(f"zone{self._zone}.volume=level-up")

    def volume_down(self) -> None:
        """Decrease volume by 1 step."""
        self.command(f"zone{self._zone}.volume=level-down")

    def mute_volume(self, mute: bool) -> None:
        """Mute (true) or unmute (false) media player."""
        if mute:
            self.command(f"zone{self._zone}.muting=on")
        else:
            self.command(f"zone{self._zone}.muting=off")

    def turn_on(self) -> None:
        """Turn the media player on."""
        self.command(f"zone{self._zone}.power=on")

    def select_source(self, source: str) -> None:
        """Set the input source."""
        if self.source_list and source in self.source_list:
            source = self._reverse_mapping[source]
        self.command(f"zone{self._zone}.selector={source}")
