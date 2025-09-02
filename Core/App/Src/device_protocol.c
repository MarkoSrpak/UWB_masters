/*
 * device_protocol.c
 *
 *  Created on: Jun 14, 2025
 *      Author: Marko Srpak
 */


/*--------------------------- INCLUDES ---------------------------------------*/
#include "device_protocol.h"
/*--------------------------- MACROS AND DEFINES -----------------------------*/
// Delay between frames, in UWB microseconds
#define POLL_TX_TO_RESP_RX_DLY_UUS 240
// Receive response timeout
#define RESP_RX_TIMEOUT_UUS 400

#define ALL_MSG_SN_IDX 2
/*--------------------------- TYPEDEFS AND STRUCTS ---------------------------*/
static const dwt_config_t config = {
    5,                /* Channel number. */
    DWT_PLEN_128,     /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    9,                /* TX preamble code. Used in TX only. */
    9,                /* RX preamble code. Used in RX only. */
    1,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (129 + 8 - 8),    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0       /* PDOA mode off */
};

// static uwb_device_t uwb_devices[5] = {0};
static const uint16_t default_ant_dly = 16385;
static uint8_t frame_seq_nb = 0;
/*--------------------------- STATIC FUNCTION PROTOTYPES ---------------------*/
static uint32_t hash_fnv1a(uint8_t *data, size_t len);
/*--------------------------- VARIABLES --------------------------------------*/

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. */
extern dwt_txconfig_t txconfig_options;

static const uwb_device_t deviceTable[]={
	{
		.partID=0,
		.lotID=0,
		.deviceHash=0x1FC5135C,
		.panID=0xABCD,
		.address16=0x0001,
		.device_id=1,
		.device_type=ANCHOR,
		.is_serial=true,
		.is_initialized=false,
		.config={},
		.tx_ant_dly=default_ant_dly,
		.rx_ant_dly=default_ant_dly,
		.coord={.x=0.0f,.y=0.0f,.z=0.0f}
	},
	{
		.partID=0,
		.lotID=0,
		.deviceHash=0xF059DE36,
		.panID=0xABCD,
		.address16=0x0002,
		.device_id=2,
		.device_type=ANCHOR,
		.is_serial=false,
		.is_initialized=false,
		.config={},
		.tx_ant_dly=default_ant_dly,
		.rx_ant_dly=default_ant_dly,
		.coord={.x=0.0f,.y=0.0f,.z=0.0f}
	},
	{
		.partID=0,
		.lotID=0,
		.deviceHash=0x1A0AB824,
		.panID=0xABCD,
		.address16=0x0003,
		.device_id=3,
		.device_type=ANCHOR,
		.is_serial=false,
		.is_initialized=false,
		.config={},
		.tx_ant_dly=default_ant_dly,
		.rx_ant_dly=default_ant_dly,
		.coord={.x=0.0f,.y=0.0f,.z=0.0f}
	},
	{
		.partID=0,
		.lotID=0,
		.deviceHash=0x4BB919FB,
		.panID=0xABCD,
		.address16=0x0004,
		.device_id=4,
		.device_type=TAG,
		.is_serial=false,
		.is_initialized=false,
		.config={},
		.tx_ant_dly=default_ant_dly,
		.rx_ant_dly=default_ant_dly,
		.coord={.x=0.0f,.y=0.0f,.z=0.0f}
	},
	{
		.partID=0,
		.lotID=0,
		.deviceHash=0,
		.panID=0xABCD,
		.address16=0x0005,
		.device_id=5,
		.device_type=TAG,
		.is_serial=false,
		.is_initialized=false,
		.config={},
		.tx_ant_dly=default_ant_dly,
		.rx_ant_dly=default_ant_dly,
		.coord={.x=0.0f,.y=0.0f,.z=0.0f}
	}

};

static uint8_t tx_msg[118];
static uint8_t rx_msg[FRAME_LEN_MAX];
/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;

/*--------------------------- STATIC FUNCTIONS -------------------------------*/
static uint32_t hash_fnv1a(uint8_t *data, size_t len) {
    uint32_t hash = 0x811c9dc5;
    for (size_t i = 0; i < len; i++) {
        hash ^= data[i];
        hash *= 0x01000193;
    }
    return hash;
}
/*--------------------------- GLOBAL FUNCTIONS -------------------------------*/
uwb_result_e uwb_device_init(uwb_device_t *uwb_device)
{
	if(NULL == uwb_device){
		return UWB_INVALID_PARAM;
	}

	printf("Start device initialization\r\n");
    /* Configure SPI rate, DW3000 supports up to 38 MHz */
    port_set_dw_ic_spi_fastrate();

    /* Reset and initialize DW chip. */
    reset_DWIC(); /* Target specific drive of RSTn line into DW3000 low for a period. */

    Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

    /* Probe for the correct device driver. */
    dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);

    while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */ { };
    if (dwt_initialise(DWT_READ_OTP_PID | DWT_READ_OTP_LID) == DWT_ERROR)
    {
    	printf("Initialization failed\r\n");
        return UWB_NOT_INITIALIZED;
    }

    uint32_t partID = dwt_getpartid();
	uint64_t lotID = dwt_getlotid();
	uint8_t buf[12];
	memcpy(buf, &partID, 4);
	memcpy(buf + 4, &lotID, 8);
	uint32_t deviceHash = hash_fnv1a(buf, 12);

	printf("partID: 0x%08lX\r\n", partID);
	printf("lotID: 0x%016llX\r\n", (unsigned long long)lotID);
	printf("deviceHash: 0x%08lX\r\n", deviceHash);

	for(uint8_t i = 0; i < sizeof(deviceTable) / sizeof(deviceTable[0]); i++)
	{
		if(deviceTable[i].deviceHash == deviceHash)
		{
			*uwb_device = deviceTable[i];
		}
	}
	if(uwb_device->deviceHash == 0)
	{
		return UWB_DEVICE_NOT_FOUND;
	}

	uwb_device->partID = partID;
	uwb_device->lotID = lotID;
	uwb_device->config = config;

    /* Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards. */
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    /* Configure DW IC*/
	/* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
	if (dwt_configure(&uwb_device->config))
	{
		printf("Configuration failed\r\n");
		return UWB_NOT_CONFIGURED;
	}

	/* Configure the TX spectrum parameters (power, PG delay and PG count) */
	dwt_configuretxrf(&txconfig_options);

	/* Apply default antenna delay value. See NOTE 2 below. */
	dwt_setrxantennadelay(uwb_device->rx_ant_dly);
	dwt_settxantennadelay(uwb_device->tx_ant_dly);

	// Ovo vjerojatno maknuti i na drugi nacin handelati
	/* Set expected response's delay and timeout.
	 * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
	//dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
	// dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

	/* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
	 * Note, in real low power applications the LEDs should not be used. */
	dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

	uwb_device->is_initialized = true;
	tx_msg[0] = 0x41;
	tx_msg[1] = 0x88;

	return UWB_OK;

}

uwb_result_e uwb_send_msg(const uwb_device_t *uwb_device, uint16_t target_device_address, const uwb_msg_t* uwb_msg, uint8_t mode)
{
    if (uwb_msg == NULL) {
        return UWB_INVALID_PARAM;
    }

    // Send the message as raw bytes
    return uwb_send_payload(
        uwb_device,
        target_device_address,
        (const uint8_t*)uwb_msg,
        sizeof(uwb_msg_t),
        mode
    );
}

uwb_result_e uwb_send_payload(const uwb_device_t *uwb_device, uint16_t target_device_address, const uint8_t* data, uint32_t data_size, uint8_t mode)
{
    if (uwb_device == NULL || data == NULL || !uwb_device->is_initialized) {
        return UWB_INVALID_PARAM;
    }

    // Total size: header (9 bytes) + payload + check sum
    uint32_t total_size = 9 + data_size + 2;

    if (total_size > 127) { // IEEE 802.15.4 max payload is 127 bytes total
        return UWB_INVALID_PARAM;
    }

    // sequence number
    tx_msg[ALL_MSG_SN_IDX] = frame_seq_nb++;

    // PAN ID (2 bytes)
    memcpy(&tx_msg[3], &uwb_device->panID, sizeof(uint16_t));

    // Destination address (2 bytes)
    memcpy(&tx_msg[5], &target_device_address, sizeof(uint16_t));

    // Source address (2 bytes)
    memcpy(&tx_msg[7], &uwb_device->address16, sizeof(uint16_t));

    // Payload (starting at byte 9)
    memcpy(&tx_msg[9], data, data_size);

    // Zero 2 bytes after payload (used for check sum that is automatically added by DW IC)
	tx_msg[9 + data_size] = 0;
	tx_msg[10 + data_size] = 0;

    // Clear TX frame sent event
    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
    dwt_writetxdata(total_size, tx_msg, 0); // Offset 0
    dwt_writetxfctrl(total_size, 0, 1);     // Offset 0, ranging frame

    // Start transmission
    dwt_starttx(mode);
#if 0
    printf("TX RAW [%lu bytes]:", total_size);
    for (uint16_t i = 0; i < total_size; i++) {
        printf(" %02X", tx_msg[i]);
    }
    printf("\n\r");
#endif

    return UWB_OK;
}

uwb_result_e uwb_receive_poll(uwb_device_t *uwb_device, uint16_t *sender_device_address, uint8_t* data, uint32_t max_data_size, uint32_t* received_size)
{
    if (uwb_device == NULL || data == NULL || sender_device_address == NULL || !uwb_device->is_initialized) {
        return UWB_INVALID_PARAM;
    }

    // Enable RX mode immediately
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    // Wait for frame received or error/timeout
    waitforsysstatus(&status_reg, NULL, (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR), 0);

    if (status_reg & DWT_INT_RXFCG_BIT_MASK)
    {
        uint16_t frame_len;

        // Clear RX frame received flag
        dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

        // Read frame length and check buffer size
        frame_len = dwt_getframelength(0);
        if (frame_len > sizeof(rx_msg)) {
            return UWB_MEMORY_ERROR;
        }

        // Read frame into rx_buffer
        dwt_readrxdata(rx_msg, frame_len, 0);

#if 0
    printf("RX RAW [%u bytes]:", frame_len);
    for (uint16_t i = 0; i < frame_len; i++) {
        printf(" %02X", rx_msg[i]);
    }
    printf("\n\r");
#endif

        // Validate Frame Control bytes, should all be the same accross all msg
        if (rx_msg[0] != tx_msg[0] || rx_msg[1] != tx_msg[1]) {
            return UWB_WRONG_ADDRESS;
        }

        // Validate PAN ID (bytes 3–4)
        uint16_t received_panID;
        memcpy(&received_panID, &rx_msg[3], sizeof(uint16_t));
        if (received_panID != uwb_device->panID) {
            return UWB_WRONG_ADDRESS;
        }

        // Validate destination address (bytes 5–6)
        uint16_t dest_addr;
        memcpy(&dest_addr, &rx_msg[5], sizeof(uint16_t));
        if (dest_addr != uwb_device->address16 && dest_addr != 0x0000) {
            return UWB_WRONG_ADDRESS;
        }

        // Extract sender address (bytes 7–8)
        memcpy(sender_device_address, &rx_msg[7], sizeof(uint16_t));

        // Extract payload (starting at byte 9, before 2-byte checksum)
        uint32_t payload_size = frame_len - 9 - 2;
        if (payload_size > max_data_size) {
            return UWB_MEMORY_ERROR;
        }

        memcpy(data, &rx_msg[9], payload_size);
        *received_size = payload_size;
        return UWB_OK;
    }

    return UWB_TIMEOUT;
}
