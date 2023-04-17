"""Support for Onkyo Receivers."""
from __future__ import annotations

# to parse http headers
from email.parser import BytesParser
import hashlib
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
    | MediaPlayerEntityFeature.STOP
    | MediaPlayerEntityFeature.PAUSE
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

# Send custom command to receiver and expect response
# example yaml
# service: media_player.send_cmd
# data:
#  cmd_resp:
#    "NJAREQ" :  "NJA"
# target:
#  entity_id: media_player.receiverdev

ATTR_CMD_RESP = "cmd_resp"
ONKYO_SEND_CMD_SCHEMA = vol.Schema(
    {
        vol.Required(ATTR_ENTITY_ID): cv.entity_ids,
        # a list of CMD and RESP
        # send CMD and wait until result appears
        # if RESP is 'None' do not await result
        vol.Required(ATTR_CMD_RESP): {cv.string: cv.string},
    }
)

SERVICE_SEND_CMD = "send_cmd"

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
            if service.service == SERVICE_SEND_CMD:
                device.service_send_cmd(service.data[ATTR_CMD_RESP])

    hass.services.register(
        DOMAIN,
        SERVICE_SELECT_HDMI_OUTPUT,
        service_handle,
        schema=ONKYO_SELECT_OUTPUT_SCHEMA,
    )

    hass.services.register(
        DOMAIN,
        SERVICE_SEND_CMD,
        service_handle,
        schema=ONKYO_SEND_CMD_SCHEMA,
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

    # maps an unsolicited message of the receiver
    # to an attribute to update
    # this is for simple commands that include status directly
    # after the three letter code  (1)
    # result will be stored as self._attr_(2)
    ATTR_MAP = {
        "NTI": "media_title",
        "NAT": "media_artist",
        "NAL": "media_album_name",
        "ATI": "media_title",  # airplay is special ... has its own set of codes
        "AAT": "media_artist",
        "AAL": "media_album_name",
    }

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

        self.net_play_unknown = True  # set to true we need to poll for current status
        # internal url that should be served from async_get_browse_image
        # assumes only one item at a time
        self.media_image_url_internal = None
        self._attr_media_image_hash = None

    def process_pending_messages(self) -> None:
        """Receiver sends autonomous updates, try to use them instead of discarding."""

        # process incoming messages, command would otherwise drop them
        while True:
            try:
                # only get messages that are already received, don't block
                rx_msg = None
                rx_msg = self._receiver.get(False)
                if not rx_msg:
                    break
            except (ValueError, OSError, AttributeError, AssertionError) as my_ex:
                if self._receiver.command_socket:
                    self._receiver.command_socket = None
                    _LOGGER.debug("Proess pending resulted in exception:")
                    _LOGGER.debug(str(my_ex))
                    _LOGGER.debug("Resetting connection to %s", self.name)
                else:
                    _LOGGER.info(
                        "%s is disconnected. Attempting to reconnect", self.name
                    )
            self.parse_message(rx_msg)

    def command(self, command):
        """Run an eiscp command and catch connection errors."""
        # process incoming messages, command would otherwise drop them
        self.process_pending_messages()
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
        # parse result (note what was dropped while waiting for result is still missing)
        self.parse_message(result)
        return result

    def raw(self, command):
        """Run an eiscp raw command and catch connection errors."""
        # process incoming messages, command would otherwise drop them
        self.process_pending_messages()
        try:
            # send the actual command
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
        # parse result (note what was dropped while waiting for result is still missing)
        self.parse_message(result)
        return result

    def send(self, command):
        """Run a raw eiscp send without any confirmation check but  catch connection errors."""
        # no need to process pending messages, this is just a send, does not drain RX socket
        # typically called with a filter_for_message (which then will get the pending  messages)
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
        # _LOGGER.debug("Sent cmd (no resp. check) %s", command)

    def calc_media_hash(self) -> str:
        """To identify what is playing, calculate hash."""
        # need someo hash to identify music / image
        # use combined text properties
        text_id = ""
        if self._attr_media_album_artist:
            text_id += self._attr_media_album_artist
        if self._attr_media_album_artist:
            text_id += self._attr_media_album_artist
        if self._attr_media_album_name:
            text_id += self._attr_media_album_name
        if self._attr_media_artist:
            text_id += self._attr_media_artist
        if self._attr_media_title:
            text_id += self._attr_media_title
        if not text_id:
            return ""
        return hashlib.sha256(text_id.encode("utf-8")).hexdigest()[:16]

    def parse_message(self, msg) -> None:
        """Parse an incoming message and update self._attr* according to MAP_ATTR."""

        # for some reason if called from play_media,
        # resulting in error TypeError: 'NoneType' object is not subscriptable
        # indicating we are getting a msg of None, protect against None ...
        if not msg:
            return

        # NOTE first do a hard coded version with each individual one, then we find something better
        _LOGGER.debug(msg)

        # if simple translation, just store
        try:
            attr_name = self.ATTR_MAP[msg[:3]]
            setattr(self, f"_attr_{attr_name}", msg[3:])
            return
        # exception will happen on all unhandled message type (normal ... )
        except KeyError:
            pass

        if msg[:3] == "NJA":
            hash_id = self.calc_media_hash()
            self._attr_media_image_hash = hash_id
            if msg[3:5] == "2-":
                self._attr_media_content_id = hash_id

                # only if we got some identify attributes (and hash can be computed)
                # otherwise new message should come once we do have dome title/artist/album info
                if not hash_id:
                    return
                # encode content type and media id in URL so that it
                # comes back to the async_get_browse_image call to fix errors
                # for direct, instead use msg[5:] + "?" + self._attr_media_image_hash
                self._attr_media_image_url = self.get_browse_image_url(
                    str(self._attr_media_content_type), self._attr_media_content_id
                )
                # buffer the actual URL locally to use in async_get_browse_image
                # + "?" + self._attr_media_image_hash
                self.media_image_url_internal = msg[5:]
                return

            # getting image for other than URL is not supported
            self._attr_media_image_url = None
            return

        # both net and airplay have same coding but different base code
        if msg[:3] == "NST" or msg[:3] == "AST":
            state_flag = msg[3:4]
            if state_flag == "S":  # STOP
                self._attr_state = MediaPlayerState.IDLE
                self._attr_media_content_type = MediaType.CHANNEL
                # turn off media info
                self._attr_media_album_artist = None
                self._attr_media_album_name = None
                self._attr_media_artist = None
                self._attr_media_title = None
                self._attr_media_image_url = None
                return
            if state_flag == "P":  # Play
                self._attr_state = MediaPlayerState.PLAYING
                self._attr_media_content_type = MediaType.MUSIC

                return
            if state_flag == "p":  # Pause
                self._attr_state = MediaPlayerState.PAUSED
                self._attr_media_content_type = MediaType.MUSIC
                return

        # we do not evaluate timing information, However, if the receiver
        # sends timing, it means it is playing.
        # This helps as it does not necessarily send out a NSTP-- message sometimes
        # not reliable, there is a nother time stamp message just after stopping.
        # would need an ignore counter ... too much
        # can't get it via # NPB12211000 either ...
        # if msg[:3] == "NTM":
        #    if self._attr_state != MediaPlayerState.PLAYING:
        #        self._attr_state = MediaPlayerState.PLAYING
        #        self._attr_media_content_type = MediaType.MUSIC
        #    return

    def filter_for_message(self, command):
        """Wait until a message that starts with command is received or timeout."""
        try:
            start = time.time()
            while True:
                candidate = self._receiver.get(0.05)
                # process the message independent of what the higher level logic wants
                self.parse_message(candidate)
                # does the message start with what we are lookig for?.
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

        # explicitly process any autonomous updates
        self.process_pending_messages()

        status = self.command("system-power query")

        if not status:
            return
        if status[1] == "on":
            # don't overwrite what we got from parsing the fly by messages
            # they differentiate more finely for the player state
            # so only do transition to on
            if self._attr_state == MediaPlayerState.OFF:
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
            self._attr_sound_mode = self._sound_modes["all"]
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

        # do this only after the initial source assignment
        # do we need to poll for the net player state?
        if self.net_play_unknown:
            # safety check that we ar in the network mode
            if self._attr_source in self._reverse_mapping_net:
                # poll if we are playing now
                self.send("NSTQSTN")
                self.filter_for_message("NST")
            self.net_play_unknown = False

        # did we get a new album art according to title/artist/album ... info
        hash_id = self.calc_media_hash()
        # _attr_media_image_hash is set when album art is received
        # if both are out of sync, try to update once
        if hash_id != self._attr_media_image_hash:
            # request a new album art, note this is
            # not NJAQSTN, which queries how album art is transferred
            # but NJAREQ, which queries the album art itself.
            self.raw("NJAREQ")
            # the above should set the new ID, but if it does not,
            # leave it at a single attempt
            self._attr_media_image_hash = hash_id

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

    def media_play(self) -> None:
        """User pressed the play button."""
        # does only work in network mode (nothing else to control otherwise)
        if self._attr_source in self._reverse_mapping_net:
            # only do a "oneway" send don't wait for result for now
            # (it will be updated on the next poll)
            self.send("NTCPLAY")
            self.net_play_unknown = True

    def media_pause(self) -> None:
        """User pressed the pause button."""
        # does only work in network mode (nothing else to control otherwise)
        if self._attr_source in self._reverse_mapping_net:
            self.send("NTCPAUSE")
            self.net_play_unknown = True

    def media_stop(self) -> None:
        """User pressed the stop button."""
        # does only work in network mode (nothing else to control otherwise)
        if self._attr_source in self._reverse_mapping_net:
            self.send("NTCSTOP")
            self.net_play_unknown = True

    def play_media(
        self, media_type: MediaType | str, media_id: str, **kwargs: Any
    ) -> None:
        """Play radio station by preset number triggered from the play_media service call."""
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

    # service to send custom commands to receiver and await response
    def service_send_cmd(self, cmd_resp_list) -> None:
        """Service call to send custom command and expect result."""
        for cmd, resp in cmd_resp_list.items():
            _LOGGER.info("Exec user cmd '%s' waiting for response '%s'", cmd, resp)
            self.send(cmd)
            if resp and resp != "None":
                self.filter_for_message(resp)

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

    # serve local album art (and fix)
    async def async_get_browse_image(
        self,
        media_content_type: MediaType | str,
        media_content_id: str,
        media_image_id: str | None = None,
    ) -> tuple[bytes | None, str | None]:
        """Serve (and fix) Onkyo album art.  Returns (content, content_type)."""

        content, content_type = (None, None)

        if media_content_type == MediaType.MUSIC and media_content_id:
            # use buffered URL
            url = self.media_image_url_internal
            if not url:
                return content, content_type

            # use original call to get the image
            content, content_type = await self._async_fetch_image(url)

            # Onkyo contains the content_type as part of the body, lets try to extract it
            if content and not content_type:
                # check if the content type is part of the body
                content_header, content_data = content.split(b"\n\n", 1)
                if content_header and content_data:
                    headers_parsed = BytesParser().parsebytes(content_header)
                    content_type = headers_parsed["Content-type"]
                    content = content_data
                else:
                    _LOGGER.error("NO Content Type for %s", url)

        return content, content_type


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
