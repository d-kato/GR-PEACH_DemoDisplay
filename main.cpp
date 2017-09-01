
#include "mbed.h"
#include "EasyAttach_CameraAndLCD.h"
#include "USBHostSerial.h"
#include "JPEG_Converter.h"

/* Video input and LCD layer 0 output */
#define GRAPHICS_FORMAT        (DisplayBase::GRAPHICS_FORMAT_YCBCR422)
#define WR_RD_WRSWA            (DisplayBase::WR_RD_WRSWA_32_16BIT)
#define DATA_SIZE_PER_PIC      (2u)

/*! Frame buffer stride: Frame buffer stride should be set to a multiple of 32 or 128
    in accordance with the frame buffer burst transfer mode. */
#define VIDEO_PIXEL_HW         LCD_PIXEL_WIDTH   /* QVGA */
#define VIDEO_PIXEL_VW         LCD_PIXEL_HEIGHT  /* QVGA */

#define FRAME_BUFFER_STRIDE    (((VIDEO_PIXEL_HW * DATA_SIZE_PER_PIC) + 31u) & ~31u)
#define FRAME_BUFFER_HEIGHT    (VIDEO_PIXEL_VW)

DisplayBase Display;
static USBHostSerial serial;

#define RESULT_BUFFER_BYTE_PER_PIXEL  (2u)
#define RESULT_BUFFER_STRIDE          (((VIDEO_PIXEL_HW * RESULT_BUFFER_BYTE_PER_PIXEL) + 31u) & ~31u)

#if defined(__ICCARM__)
#pragma data_alignment=32
static uint8_t user_frame_buffer0[FRAME_BUFFER_STRIDE * FRAME_BUFFER_HEIGHT]@ ".mirrorram";
#pragma data_alignment=32
static uint8_t user_frame_buffer_result[RESULT_BUFFER_STRIDE * FRAME_BUFFER_HEIGHT]@ ".mirrorram";
#pragma data_alignment=32
static uint8_t JpegBuffer[1024 * 64]@ ".mirrorram";
#else
static uint8_t user_frame_buffer0[FRAME_BUFFER_STRIDE * FRAME_BUFFER_HEIGHT]__attribute((section("NC_BSS"),aligned(32)));
static uint8_t user_frame_buffer_result[RESULT_BUFFER_STRIDE * FRAME_BUFFER_HEIGHT]__attribute((section("NC_BSS"),aligned(32)));
static uint8_t JpegBuffer[1024 * 64]__attribute((section("NC_BSS"),aligned(32)));
#endif

static bool draw_square = false;

void ClearSquare(void) {
    if (draw_square) {
        memset(user_frame_buffer_result, 0, sizeof(user_frame_buffer_result));
        draw_square = false;
    }
}

void DrawSquare(int x, int y, int w, int h, uint32_t const colour) {
    int idx_base;
    int wk_idx;
    int i;
    uint8_t coller_pix[RESULT_BUFFER_BYTE_PER_PIXEL];  /* ARGB4444 */

    idx_base = (x + (VIDEO_PIXEL_HW * y)) * RESULT_BUFFER_BYTE_PER_PIXEL;

    /* Select color */
    coller_pix[0] = (colour >> 8) & 0xff;  /* 4:Green 4:Blue */
    coller_pix[1] = colour & 0xff;         /* 4:Alpha 4:Red  */

    /* top */
    wk_idx = idx_base;
    for (i = 0; i < w; i++) {
        user_frame_buffer_result[wk_idx++] = coller_pix[0];
        user_frame_buffer_result[wk_idx++] = coller_pix[1];
    }

    /* middle */
    for (i = 1; i < (h - 1); i++) {
        wk_idx = idx_base + (VIDEO_PIXEL_HW * RESULT_BUFFER_BYTE_PER_PIXEL * i);
        user_frame_buffer_result[wk_idx + 0] = coller_pix[0];
        user_frame_buffer_result[wk_idx + 1] = coller_pix[1];
        wk_idx += (w - 1) * RESULT_BUFFER_BYTE_PER_PIXEL;
        user_frame_buffer_result[wk_idx + 0] = coller_pix[0];
        user_frame_buffer_result[wk_idx + 1] = coller_pix[1];
    }

    /* bottom */
    wk_idx = idx_base + (VIDEO_PIXEL_HW * RESULT_BUFFER_BYTE_PER_PIXEL * (h - 1));
    for (i = 0; i < w; i++) {
        user_frame_buffer_result[wk_idx++] = coller_pix[0];
        user_frame_buffer_result[wk_idx++] = coller_pix[1];
    }
    draw_square = true;
}


static void Start_LCD_Display(void) {
    DisplayBase::rect_t rect;

    rect.vs = 0;
    rect.vw = LCD_PIXEL_HEIGHT;
    rect.hs = 0;
    rect.hw = LCD_PIXEL_WIDTH;
    Display.Graphics_Read_Setting(
        DisplayBase::GRAPHICS_LAYER_0,
        (void *)user_frame_buffer0,
        FRAME_BUFFER_STRIDE,
        GRAPHICS_FORMAT,
        WR_RD_WRSWA,
        &rect
    );
    Display.Graphics_Start(DisplayBase::GRAPHICS_LAYER_0);

    // GRAPHICS_LAYER_2
    memset(user_frame_buffer_result, 0, sizeof(user_frame_buffer_result));

    rect.vs = 0;
    rect.vw = VIDEO_PIXEL_VW;
    rect.hs = 0;
    rect.hw = VIDEO_PIXEL_HW;
    Display.Graphics_Read_Setting(
        DisplayBase::GRAPHICS_LAYER_2,
        (void *)user_frame_buffer_result,
        RESULT_BUFFER_STRIDE,
        DisplayBase::GRAPHICS_FORMAT_ARGB4444,
        DisplayBase::WR_RD_WRSWA_32_16BIT,
        &rect
    );
    Display.Graphics_Start(DisplayBase::GRAPHICS_LAYER_2);

    Thread::wait(50);
    EasyAttach_LcdBacklight(true);
}

static bool jcu_end = true;

static char detect_str[32];

static void JcuDecodeCallBackFunc(JPEG_Converter::jpeg_conv_error_t err_code) {
    jcu_end = true;
}

int main(void) {
    // Initialize the background to black
    for (uint32_t i = 0; i < sizeof(user_frame_buffer0); i += 2) {
        user_frame_buffer0[i + 0] = 0x10;
        user_frame_buffer0[i + 1] = 0x80;
    }

    EasyAttach_Init(Display);
    Start_LCD_Display();

    const unsigned char head_data[] = {0xFF, 0xFF, 0xAA, 0x55};
    int index = 0;
    int RcvImageSize;
    int data_type;
    JPEG_Converter  Jcu;

    while (1) {
        /* try to connect a serial device */
        while (!serial.connect()) {
            Thread::wait(500);
        }
        serial.baud(9600);
        serial.format();
        while (1) {
            if (!serial.connected()) {
                break;
            }
            // 先頭検索する（先頭がFF FF AA 55の４バイトになっている所を探す）
            if (serial.getc() == head_data[index]){
                index++;
                if (index >= sizeof(head_data)){
                    // 先頭検出した直後状態の状態。データのヘッダ部分を読み出す（ヘッダは８バイト）
                    (void)serial.getc(); // 最初の４バイトは、リザーブデータなので読み捨てる
                    (void)serial.getc();
                    (void)serial.getc();
                    data_type = serial.getc();

                    RcvImageSize = serial.getc(); //次の４バイトは、JPEGデータサイズ（リトルエンディアン）
                    RcvImageSize += (serial.getc()) * 0x100;
                    RcvImageSize += (serial.getc()) * 0x10000;
                    RcvImageSize += (serial.getc()) * 0x1000000;

                    if (data_type == 0) {
                        serial.readBuf((char *)JpegBuffer, RcvImageSize, 1000);
                        // JPEGデータを表示する処理

                        while (jcu_end == false) {
                            Thread::wait(1);
                        }

                        JPEG_Converter::bitmap_buff_info_t bitmap_buff_info;
                        JPEG_Converter::decode_options_t   decode_options;

                        bitmap_buff_info.width              = VIDEO_PIXEL_HW;
                        bitmap_buff_info.height             = VIDEO_PIXEL_VW;
                        bitmap_buff_info.format             = JPEG_Converter::WR_RD_YCbCr422;
                        bitmap_buff_info.buffer_address     = (void *)user_frame_buffer0;

                        decode_options.output_swapsetting    = JPEG_Converter::WR_RD_WRSWA_32_16_8BIT;
                        decode_options.p_DecodeCallBackFunc = &JcuDecodeCallBackFunc;

                        jcu_end = false;
                        if (!Jcu.decode((void *)JpegBuffer, &bitmap_buff_info, &decode_options) == JPEG_Converter::JPEG_CONV_OK) {
                            jcu_end = true;
                        }
                    } else if (data_type == 1) {
                        // 四角を描画
                        memset(detect_str, 0, sizeof(detect_str));
                        if (RcvImageSize > sizeof(detect_str)) {
                            RcvImageSize = sizeof(detect_str);
                        }
                        serial.readBuf(detect_str, RcvImageSize, 1000);

                        int wk_buf[4];
                        int idx = 0;
                        int start_pos = 0;

                        for (int i = 0; (i < RcvImageSize + 1) && (idx < 4); i++) {
                            if ((detect_str[i] == ',') || (detect_str[i] == '\0')) {
                                detect_str[i] = '\0';
                                wk_buf[idx++] = atoi(&detect_str[start_pos]);
                                start_pos = i + 1;
                            }
                        }

                        ClearSquare();
                        DrawSquare(wk_buf[0], wk_buf[1], wk_buf[2], wk_buf[3], 0x0000f0f0);

                    }
                    index = 0;
                }
            } else {
                index = 0;
            }
        }
    }
}

