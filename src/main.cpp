#include <avr/io.h>
#include <util/delay.h>
#include "nvmalloc.h"

#define MAX_ALTERNATE 3
#define EP_SIZE 64
#define BL_SIZE 0x600
#define U16LE(_u16) ((_u16) & 0xFF), (((_u16) >> 8) & 0xFF)
#define MIN(_a, _b) ((_a)<(_b) ? (_a) : (_b))

const BOOTROW_VAR uint8_t DeviceDescriptor[] = {
    18,             /* bLength */
    0x01,           /* bDescriptorType = Device */
    U16LE(0x0200),  /* bcdUSB */
    0x00,           /* bDeviceClass */
    0x00,           /* bDeviceSubClass */
    0x00,           /* bDeviceProtocol */
    64,             /* bMaxPacketSize0 */
    U16LE(0xCA75),  /* idVendor */
    U16LE(0xF000),  /* idProduct */
    U16LE(0x0100),  /* bcdDevice */
    0x01,           /* iManufacturer */
    0x02,           /* iProduct */
    0x03,           /* iSerialNumber */
    0x01,           /* bNumConfigurations */
};
const BOOTROW_VAR uint8_t ConfigDescriptor[] = {
    /* CONFIG DESC */
    9,              /* bLength */
    0x02,           /* bDescriptorType = Configuration */
    U16LE(9 + 9 + 9 * 3), /* wTotalLength */
    1,              /* bNumInterfaces */
    1,              /* bConfigurationValue */
    0x04,           /* iConfiguration */
    0xC0,           /* bmAttributes = Self-powered */
    50,             /* bMaxPower = 100 mA */
    /* DFU INTERFACE DESC Alt 0 */
    9,              /* bLength */
    0x04,           /* bDescriptorType = Interface */
    0,              /* bInterfaceNumber = 0 */
    0,              /* bAlternateSetting = 0 */
    0,              /* bNumEndpoints = 0 */
    0xFE,           /* bInterfaceClass = AppSpecific */
    0x01,           /* bInterfaceSubClass = DFU */
    0x02,           /* bInterfaceProtocol = DFU */
    5,              /* iInterface */
    /* DFU INTERFACE DESC Alt 1 */
    9,              /* bLength */
    0x04,           /* bDescriptorType = Interface */
    0,              /* bInterfaceNumber = 0 */
    1,              /* bAlternateSetting = 0 */
    0,              /* bNumEndpoints = 0 */
    0xFE,           /* bInterfaceClass = AppSpecific */
    0x01,           /* bInterfaceSubClass = DFU */
    0x02,           /* bInterfaceProtocol = DFU */
    6,              /* iInterface */
    /* DFU INTERFACE DESC Alt 2 */
    9,              /* bLength */
    0x04,           /* bDescriptorType = Interface */
    0,              /* bInterfaceNumber = 0 */
    2,              /* bAlternateSetting = 0 */
    0,              /* bNumEndpoints = 0 */
    0xFE,           /* bInterfaceClass = AppSpecific */
    0x01,           /* bInterfaceSubClass = DFU */
    0x02,           /* bInterfaceProtocol = DFU */
    7,              /* iInterface */
    /* DFU FUNC DESC 0 */
    9,              /* bLength */
    0x21,           /* bDescriptorType = DFU Func */
    0x0F,           /* bmAttributes = WillDetach | ManifestationTolerant | U | D */
    U16LE(1000),    /* wDetachTimeout */
    U16LE(64),      /* wTransferSize: we can accept much more, maybe a whole page (512 B)? */
    U16LE(0x0110),  /* bcdDFUVersion */
};
const BOOTROW_VAR uint8_t SD0_LangID[] = { 4, 3, U16LE(0x0409) };
const BOOTROW_VAR char SD1_Manufacturer[] = { 22, 3, 'B', 0, 'i', 0, 't', 0, 'e', 0, 'r', 0, ' ', 0, 'C', 0, 'a', 0, 't', 0, 's', 0, };                            // "Biter Cats"
const BOOTROW_VAR char SD2_4_Product_Config[] = { 28, 3, 'A', 0, 'V', 0, 'R', 0, ' ', 0, 'D', 0, 'U', 0, ' ', 0, 'T', 0, 'i', 0, 'n', 0, 'y', 0, 'B', 0, 'L', 0 }; // "AVR DU TinyBL"
const BOOTROW_VAR char SD5_Alt0[] = { 24, 3, 'F', 0, 'l', 0, 'a', 0, 's', 0, 'h', 0, '+', 0, '0', 0, 'x', 0, '6', 0, '0', 0, '0', 0};                              // "Flash+0x600"
const BOOTROW_VAR char SD6_Alt1[] = { 14, 3, 'E', 0, 'E', 0, 'P', 0, 'R', 0, 'O', 0, 'M', 0 };                                                                     // "EEPROM"
const BOOTROW_VAR char SD5_Alt2[] = { 16, 3, 'U', 0, 'S', 0, 'E', 0, 'R', 0, 'R', 0, 'O', 0, 'W', 0 };                                                             // "USERROW"
const BOOTROW_VAR void *const BOOTROW_VAR SDTable[] = {
    SD0_LangID,
    SD1_Manufacturer,
    SD2_4_Product_Config,
    nullptr,
    SD2_4_Product_Config,
    SD5_Alt0,
    SD6_Alt1,
    SD5_Alt2
};

__attribute__((noinit)) USB_EP_PAIR_t EP0;
__attribute__((noinit, aligned(2))) volatile uint8_t SetupBuffer[8];
__attribute__((noinit, aligned(2))) volatile uint8_t PacketBuffer[64];
__attribute__((noinit)) struct __attribute__((packed)) {
    uint8_t RequestType;
    uint8_t Request;
    uint16_t Value;
    uint16_t Index;
    uint16_t Length;
    // Expected transaction
    enum { CXSM_SETUP, CXSM_DATA, CXSM_STATUS } Stage;
} CtrlXferSM;
__attribute__((noinit)) uint8_t InterfaceAlternate, Configuration;
__attribute__((noinit)) struct {
    enum {
        APP_IDLE,
        APP_DETACH,
        IDLE,
        DNLOAD_SYNC,
        DNBUSY,
        DNLOAD_IDLE,
        MANIFEST_SYNC,
        MANIFEST,
        MANIFEST_WAIT_RESET,
        UPLOAD_IDLE,
        ERROR
    } State;
    uint8_t *DownloadPtr;
} DfuSM;
enum DfuRequests {
    DFU_DNLOAD = 1,
    DFU_UPLOAD = 2,
    DFU_GETSTATUS = 3,
    DFU_CLRSTATUS = 4,
    DFU_GETSTATE = 5,
    DFU_ABORT = 6,
};


static void poll_for_usb_rmw(void) {
    while(USB0.INTFLAGSB & USB_RMWBUSY_bm)
        ;
}

static void xfer_in(volatile uint8_t *data, uint16_t count) {
    EP0.IN.DATAPTR = reinterpret_cast<uint16_t>(data);
    EP0.IN.CNT = count;
    EP0.IN.MCNT = 0;
    poll_for_usb_rmw();
    USB0.STATUS[0].INCLR = USB_CRC_bm | USB_UNFOVF_bm | USB_TRNCOMPL_bm | USB_EPSETUP_bm | USB_STALLED_bm | USB_BUSNAK_bm;
}

static void xfer_out(void) {
    // IN.DATAPTR used for Data Stage in both directions
    EP0.IN.DATAPTR = reinterpret_cast<uint16_t>(PacketBuffer);
    EP0.OUT.CNT = 0; // may not be necessary for non-multipacket mode
    poll_for_usb_rmw();
    USB0.STATUS[0].OUTCLR = USB_CRC_bm | USB_UNFOVF_bm | USB_TRNCOMPL_bm | USB_EPSETUP_bm | USB_STALLED_bm | USB_BUSNAK_bm;
}

static void xfer_in_data(uint8_t volatile *data, uint16_t count) {
    xfer_in(data, count);
    CtrlXferSM.Stage = CtrlXferSM.CXSM_DATA;
}

static void xfer_in_status(void) {
    xfer_in(nullptr, 0);
    CtrlXferSM.Stage = CtrlXferSM.CXSM_STATUS;
}

static void xfer_out_data(void) {
    xfer_out();
    CtrlXferSM.Stage = CtrlXferSM.CXSM_DATA;
}

static void xfer_out_status(void) {
    xfer_out();
    CtrlXferSM.Stage = CtrlXferSM.CXSM_STATUS;
}

static void stall(void) {
    EP0.IN.CTRL |= USB_DOSTALL_bm;
    EP0.OUT.CTRL |= USB_DOSTALL_bm;
}

static void xfer_in_data_from_bootrow(const uint8_t *data, uint8_t count) {
    for(uint8_t i = 0; i < MIN(count, sizeof(PacketBuffer)); i++) {
        PacketBuffer[i] = data[i];
    }
    xfer_in_data(PacketBuffer, count);
}

static void poll_for_nvm_busy_cleared(void){
    while(NVMCTRL.STATUS & (NVMCTRL_FLBUSY_bm | NVMCTRL_EEBUSY_bm))
        ;
}

static void send_serial_desc(void) {
    // Use 12 bytes from Signature Row Serial Number
    PacketBuffer[0] = 26;
    PacketBuffer[1] = 3;  // String descriptor (26 B)
    volatile uint8_t *sernum = (&SIGROW.SERNUM0);
    volatile uint8_t *pbuf = &PacketBuffer[2];
    for (uint8_t i = 0; i<12; i++) {
        // Coerce to printable region of ASCII, though the result is not truely unique
        *pbuf++ = (*sernum++ & 0x3F) + 0x30;
        *pbuf++ = 0;
    }
    xfer_in_data(PacketBuffer, 26);
}

static void handle_get_descriptor(void) {
    uint8_t desctype = CtrlXferSM.Value >> 8;
    uint8_t descidx = CtrlXferSM.Value & 0xFF;
    if(desctype == 1 /* Device */) {
        xfer_in_data_from_bootrow(DeviceDescriptor, MIN(sizeof(DeviceDescriptor), CtrlXferSM.Length));
    } else if(desctype == 2 /* Config */ /* && descidx == 0 */) {
        xfer_in_data_from_bootrow(ConfigDescriptor, MIN(sizeof(ConfigDescriptor), CtrlXferSM.Length));
    } else if(desctype == 3 /* String */ && descidx < sizeof(SDTable) / sizeof(void *)) {
        const uint8_t *strdesc = reinterpret_cast<const uint8_t *>(SDTable[descidx]);
        if(strdesc == nullptr) {
            send_serial_desc();
        } else {
            uint8_t desclen = strdesc[0];
            xfer_in_data_from_bootrow(strdesc, MIN(desclen, CtrlXferSM.Length));
        }
    } else {
        stall();
    }
}

static void dfu_stall(void) {
    stall();
    DfuSM.State = DfuSM.ERROR;
}

static uint8_t *get_block_ptr(void) {
    uint16_t blocknum = CtrlXferSM.Value;
    if (InterfaceAlternate == 0) {
        blocknum += BL_SIZE / EP_SIZE;
        uint8_t devid1 = SIGROW.DEVICEID1;
        uint16_t max_block = ((devid1 == 0x94) ? (0x4000 / EP_SIZE) : (devid1 == 0x95) ? (0x8000 / EP_SIZE) : (0x10000 / EP_SIZE));
        if(blocknum < max_block) {
            if(blocknum < 0x8000 / EP_SIZE) {
                _PROTECTED_WRITE(NVMCTRL.CTRLB, NVMCTRL_FLMAP_SECTION0_gc);
                return reinterpret_cast<uint8_t *>(0x8000) + (blocknum * EP_SIZE);
            } else {
                _PROTECTED_WRITE(NVMCTRL.CTRLB, NVMCTRL_FLMAP_SECTION1_gc);
                return reinterpret_cast<uint8_t *>(0x0) + (blocknum * EP_SIZE);
            }
        }
    } else if(InterfaceAlternate == 1) {
        if (blocknum < EEPROM_SIZE / EP_SIZE)
            return reinterpret_cast<uint8_t*>(EEPROM_START) + (blocknum * EP_SIZE);
    } else {
        if (blocknum < sizeof(USERROW_t) / EP_SIZE)
            return reinterpret_cast<uint8_t *>(&USERROW) + (blocknum * EP_SIZE);
    }
    return nullptr;
}

static void do_download(void) {
    // Handles DNLOAD request
    // ZLP DNLOAD has special meaning to move to manifestation
    if(CtrlXferSM.Length == 0) {
        xfer_in_status();
        DfuSM.State = DfuSM.MANIFEST_SYNC;
        return;
    }
    // Otherwise, remember where to write after data stage
    uint8_t* blockaddr = get_block_ptr();
    if(!blockaddr) {
        dfu_stall();
    } else {
        xfer_out_data();
        DfuSM.DownloadPtr = blockaddr;
        DfuSM.State = DfuSM.DNLOAD_IDLE;
        // TODO: Do Erase here and enable NVM write access
        if(InterfaceAlternate == 1) {
            // EEPROM E/W
            poll_for_nvm_busy_cleared();
            _PROTECTED_WRITE_SPM(NVMCTRL_CTRLA, NVMCTRL_CMD_EEERWR_gc);
        } else {
            // Erase Page if the block is the first of a page (512 B)
            if((reinterpret_cast<uint16_t>(blockaddr) & 0x01FF) == 0) {
                poll_for_nvm_busy_cleared();
                _PROTECTED_WRITE_SPM(NVMCTRL_CTRLA, NVMCTRL_CMD_FLPER_gc);
                poll_for_nvm_busy_cleared();
                *blockaddr = 0;
                poll_for_nvm_busy_cleared();
                _PROTECTED_WRITE_SPM(NVMCTRL_CTRLA, NVMCTRL_CMD_NONE_gc);
            }
            // Flash/User Row W
            poll_for_nvm_busy_cleared();
            _PROTECTED_WRITE_SPM(NVMCTRL_CTRLA, NVMCTRL_CMD_FLWR_gc);
        }
    }
}

static void do_download_data(void) {
    // Data stage complete and we have block to program
    // This check shouldn't be necessary; just in case
    if (DfuSM.DownloadPtr) {
        // Program NVM (assuming NVMCTRL mode is appropriately set in setup stage)
        volatile uint8_t *ptr = DfuSM.DownloadPtr;
        DfuSM.DownloadPtr = nullptr;
        for(uint8_t i=0; i < EP0.OUT.CNT; i++) {
            poll_for_nvm_busy_cleared();
            *ptr++ = PacketBuffer[i];
        }
        poll_for_nvm_busy_cleared();
        _PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_NONE_gc);
        xfer_in_status();
    }
}

static void do_upload(void) {
    // Handles UPLOAD request. Simply determine the address and respond with what's there.
    uint8_t* blockaddr = get_block_ptr();
    if(!blockaddr) {
        // If out of range, reply with a ZLP.
        xfer_in_data(nullptr, 0);
        DfuSM.State = DfuSM.IDLE;
    } else {
        uint8_t xfersize = MIN(EP_SIZE, CtrlXferSM.Length);
        for (uint8_t i = 0; i < xfersize; i++)
            PacketBuffer[i] = *blockaddr++;
        xfer_in_data(PacketBuffer, xfersize);
        DfuSM.State = DfuSM.UPLOAD_IDLE;
    }
}

static void do_getstatus(uint8_t newstate, uint8_t delay_ms) {
    PacketBuffer[0] = 0; // Hardcode OK this time
    PacketBuffer[1] = delay_ms;
    PacketBuffer[2] = 0;
    PacketBuffer[3] = 0;
    PacketBuffer[4] = newstate;
    PacketBuffer[5] = 0;
    xfer_in_data(PacketBuffer, 6);
    DfuSM.State = static_cast<decltype(DfuSM.State)>(newstate);
}

static void handle_setup(void) {
    // Copy SETUP packet and clear the flags first
    for(uint8_t i = 0; i < 8; i++)
        reinterpret_cast<uint8_t *>(&CtrlXferSM)[i] = SetupBuffer[i];
    USB0.INTFLAGSB = USB_SETUP_bm;
    EP0.IN.CTRL &= ~USB_DOSTALL_bm;
    EP0.OUT.CTRL &= ~USB_DOSTALL_bm;
    poll_for_usb_rmw();
    USB0.STATUS[0].INCLR = USB_EPSETUP_bm;
    poll_for_usb_rmw();
    USB0.STATUS[0].OUTCLR = USB_EPSETUP_bm;
    // Parse SETUP packet and set up Data Stage
    if((CtrlXferSM.RequestType & 0x60) == 0x00 /* Standard requests */) {
        if(CtrlXferSM.Request == 8 /* GET_CONFIGURATION */) {
            PacketBuffer[0] = Configuration;
            xfer_in_data(PacketBuffer, 1);
        } else if(CtrlXferSM.Request == 9 /* SET_CONFIGURATION */) {
            // Technically we have to go back to Address state if Value == 0
            if(CtrlXferSM.Value <= 1) {
                Configuration = CtrlXferSM.Value;
                xfer_in_status();
            } else {
                stall();
            }
        } else if(CtrlXferSM.Request == 10 /* GET_INTERFACE */) {
            PacketBuffer[0] = InterfaceAlternate;
            xfer_in_data(PacketBuffer, 1);
        } else if(CtrlXferSM.Request == 11 /* SET_INTERFACE */) {
            if(CtrlXferSM.Value < MAX_ALTERNATE) {
                InterfaceAlternate = CtrlXferSM.Value;
                xfer_in_status();
            } else {
                stall();
            }
        } else if(CtrlXferSM.Request == 5 /* SET_ADDRESS */) {
            // Just nod now and set ADDR after this status transaction
            xfer_in_status();
        } else if(CtrlXferSM.Request == 6 /* GET_DESCRIPTOR */) {
            handle_get_descriptor();
        } else if(CtrlXferSM.Request == 0 /* GET_STATUS */) {
            // Whatever wIndex is, we can just return 0 in our case
            PacketBuffer[0] = 0;
            PacketBuffer[1] = 0;
            xfer_in_data(PacketBuffer, 2);
        } else {
            stall();
        }
    } else if((CtrlXferSM.RequestType & 0x60) == 0x20 /* Class Requests */) {
        if(CtrlXferSM.Request == DFU_GETSTATE) {
            // GETSTATE is always accepted and causes no changes to SM
            PacketBuffer[0] = DfuSM.State;
            xfer_in_data(PacketBuffer, 1);
        } else if(CtrlXferSM.Request == DFU_ABORT && DfuSM.State != DfuSM.ERROR) {
            // ABORT is also accepted by many states
            xfer_in_status();
            DfuSM.State = DfuSM.IDLE;
        } else if(DfuSM.State == DfuSM.IDLE) {
            if(CtrlXferSM.Request == DFU_DNLOAD) {
                do_download();
            } else if(CtrlXferSM.Request == DFU_UPLOAD) {
                do_upload();
            } else if(CtrlXferSM.Request == DFU_GETSTATUS) {
                do_getstatus(DfuSM.IDLE, 1);
            } else {
                dfu_stall();
            }
        } else if(DfuSM.State == DfuSM.DNLOAD_SYNC && CtrlXferSM.Request == DFU_GETSTATUS) {
            do_getstatus(DfuSM.IDLE, 10);
        } else if(DfuSM.State == DfuSM.DNLOAD_IDLE) {
            if(CtrlXferSM.Request == DFU_DNLOAD) {
                do_download();
            } else if(CtrlXferSM.Request == DFU_GETSTATUS) {
                do_getstatus(DfuSM.DNLOAD_IDLE, 1);
            } else {
                dfu_stall();
            }
        } else if(DfuSM.State == DfuSM.MANIFEST_SYNC && CtrlXferSM.Request == DFU_GETSTATUS) {
            do_getstatus(DfuSM.IDLE, 10);
        } else if(DfuSM.State == DfuSM.UPLOAD_IDLE) {
            if(CtrlXferSM.Request == DFU_UPLOAD) {
                do_upload();
            } else if(CtrlXferSM.Request == DFU_GETSTATUS) {
                do_getstatus(DfuSM.UPLOAD_IDLE, 1);
            } else {
                dfu_stall();
            }
        } else if(DfuSM.State == DfuSM.ERROR) {
            if(CtrlXferSM.Request == DFU_CLRSTATUS) {
                xfer_in_status();
                DfuSM.State = DfuSM.IDLE;
            } else if(CtrlXferSM.Request == DFU_GETSTATUS) {
                do_getstatus(DfuSM.ERROR, 1);
            } else {
                dfu_stall();
            }
        } else {
            dfu_stall();
        }
    } else {
        stall();
    }
}

static void handle_trncompl(void) {
    USB0.INTFLAGSB = USB_TRNCOMPL_bm;
    bool dir_in = EP0.IN.STATUS & USB_TRNCOMPL_bm;
    poll_for_usb_rmw();
    USB0.STATUS[0].INCLR = USB_EPSETUP_bm;
    poll_for_usb_rmw();
    USB0.STATUS[0].OUTCLR = USB_EPSETUP_bm;
    if(CtrlXferSM.Stage == CtrlXferSM.CXSM_STATUS) {
        // Just completed status stage. Generally, nothing to do except for SET_ADDRESS
        if((CtrlXferSM.RequestType & 0x60) == 0x00 /* Standard requests */ && CtrlXferSM.Request == 5 /* SET_ADDRESS */) {
            USB0.ADDR = CtrlXferSM.Value;
        }
        CtrlXferSM.Stage = CtrlXferSM.CXSM_SETUP;
    } else if(CtrlXferSM.Stage == CtrlXferSM.CXSM_DATA) {
        // Data stage completed
        if((CtrlXferSM.RequestType & 0x60) == 0x20 /* Class Requests */ && CtrlXferSM.Request == DFU_DNLOAD) {
            // DFU Download payload received.
            do_download_data();
        } else if(dir_in) {
            // Otherwise, nod with ZLP
            xfer_out_status();
        } else {
            xfer_in_status();
        }
        CtrlXferSM.Stage = CtrlXferSM.CXSM_STATUS;
    }
}

static void reset_usb(void) {
    InterfaceAlternate = 0;
    Configuration = 0;
    DfuSM.State = DfuSM.IDLE;
    EP0.IN.STATUS = USB_BUSNAK_bm;
    EP0.IN.CTRL = static_cast<uint8_t>(USB_TYPE_CONTROL_gc) | USB_BUFSIZE_DEFAULT_BUF64_gc;
    EP0.OUT.STATUS = USB_BUSNAK_bm;
    EP0.OUT.CTRL = static_cast<uint8_t>(USB_TYPE_CONTROL_gc) | USB_BUFSIZE_DEFAULT_BUF64_gc;
    EP0.OUT.DATAPTR = reinterpret_cast<uint16_t>(SetupBuffer);
    USB0.EPPTR = reinterpret_cast<uint16_t>(&EP0);
    USB0.ADDR = 0;
    USB0.INTCTRLA = 0;
    USB0.INTCTRLB = 0;
    USB0.INTFLAGSA = USB_SOF_bm | USB_SUSPEND_bm | USB_RESUME_bm | USB_RESET_bm | USB_STALLED_bm | USB_UNF_bm | USB_OVF_bm;
    USB0.INTFLAGSB = USB_TRNCOMPL_bm | USB_GNDONE_bm | USB_SETUP_bm;
}

static void bootloader_main(void) {
    // USB requires 12 MHz MCLK
    _PROTECTED_WRITE(CLKCTRL.OSCHFCTRLA, CLKCTRL_FRQSEL_24M_gc);
    SYSCFG.VUSBCTRL = SYSCFG_USBVREG_bm;
    // Initialize USB (EP0 only, no FIFO, no interrupt)
    reset_usb();
    USB0.CTRLB = USB_ATTACH_bm;
    USB0.CTRLA = USB_ENABLE_bm | (0 << USB_MAXEP_gp);
    // Main loop
    while(true) {
        if(USB0.INTFLAGSB & USB_SETUP_bm) {
            handle_setup();
        }
        if(USB0.INTFLAGSB & USB_TRNCOMPL_bm) {
            handle_trncompl();
        }
        if(USB0.INTFLAGSA & USB_RESET_bm) {
            reset_usb();
            // Run app if PC3 is deasserted when reset
            if(VPORTC.IN & 0x08) {
                // ...by device reset, to save space
                _PROTECTED_WRITE(RSTCTRL.SWRR, RSTCTRL_SWRST_bm);
            }
        }
    }
    // If we want to jump to the app, we have to restore the reset state of peripherals used.
    // As we leave bootloader by SW reset, this part is unreachable.
    // USB0.CTRLB = 0;
    // USB0.CTRLA = 0;
    // SYSCFG.VUSBCTRL = 0;
    // poll_for_nvm_busy_cleared();
    // _PROTECTED_WRITE_SPM(NVMCTRL.CTRLA, NVMCTRL_CMD_NONE_gc);
    // _PROTECTED_WRITE(NVMCTRL.CTRLB, NVMCTRL_FLMAP_SECTION3_gc);
    // _PROTECTED_WRITE(CLKCTRL.OSCHFCTRLA, CLKCTRL_FRQSEL_4M_gc);
}

__attribute__((noreturn)) int main(void) {
    // Run bootloader if PC3 is low
    PORTC.PIN3CTRL = PORT_PULLUPEN_bm;
    // 768-cycle delay (192 us @ 4 MHz) to make sure pull-up takes effect.
    asm volatile(
        "clr __tmp_reg__ \n\t"
        "inc __tmp_reg__ \n\t"
        "brne .-4        \n\t"
    );
    if(!(VPORTC.IN & 0x08))
        bootloader_main();
    PORTC.PIN3CTRL = 0;
    asm volatile("jmp %0"::"n"(BL_SIZE));
    __builtin_unreachable();
}

// Startup code
extern "C" __attribute__((naked, section(".vectors"))) void vectors(void) {
    // Original vector table in CRT object is removed by novector.ld.
    // As we don't need vectors, we immediately start minimal CRT initialization.
    // As long as we avoid .data and .bss, the only thing to do is to set r1, SP and Y.
    // (SP has reset value of RAMEND; SP may not need be set)
    asm volatile(
        "clr __zero_reg__ \n\t"
        "ldi r28, 0xFF    \n\t"
        "ldi r29, 0x7F    \n\t"
        "out 0x3D, r28    \n\t"
        "out 0x3E, r29    \n\t"
        "rjmp main        \n\t"
    );
}
