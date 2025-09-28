#include "esphome/core/log.h"
#include "it8951e.h"
#include "it8951.h"
#include "esphome/core/application.h"
#include "esphome/core/gpio.h"

namespace esphome {
namespace it8951e {

static const char *TAG = "it8951e.display";

void IT8951ESensor::write_two_byte16(uint16_t type, uint16_t cmd) {
    this->wait_busy();
    this->enable();

    this->write_byte16(type);
    this->wait_busy();
    this->write_byte16(cmd);

    this->disable();
}

uint16_t IT8951ESensor::read_word() {
    this->wait_busy();
    this->enable();
    this->write_byte16(0x1000);
    this->wait_busy();

    // dummy
    this->write_byte16(0x0000);
    this->wait_busy();

    uint8_t recv[2];
    this->read_array(recv, sizeof(recv));
    uint16_t word = encode_uint16(recv[0], recv[1]);

    this->disable();
    return word;
}

void IT8951ESensor::read_words(void *buf, uint32_t length) {
    this->wait_busy();
    this->enable();
    this->write_byte16(0x1000);
    this->wait_busy();
 
    // dummy
    this->write_byte16(0x0000);
    this->wait_busy();
 
    // Read all words in a single transaction
    this->read_array(static_cast<uint8_t *>(buf), length * 2);
 
    // The IT8951 returns data in big-endian format. Swap bytes if the MCU is little-endian.
    for (uint32_t i = 0; i < length; i++) {
        static_cast<uint16_t *>(buf)[i] = __builtin_bswap16(static_cast<uint16_t *>(buf)[i]);
    }
 
    this->disable();
}

void IT8951ESensor:: write_command(uint16_t cmd) {
    this->write_two_byte16(0x6000, cmd);
}

void IT8951ESensor::write_word(uint16_t cmd) {
    this->write_two_byte16(0x0000, cmd);
}

void IT8951ESensor::write_reg(uint16_t addr, uint16_t data) {
    this->write_command(0x0011);  // tcon write reg command
    this->wait_busy();
    this->enable();
    this->write_byte(0x0000); // Preamble
    this->wait_busy();
    this->write_byte16(addr);
    this->wait_busy();
    this->write_byte16(data);
    this->disable();
}

void IT8951ESensor::set_target_memory_addr(uint16_t tar_addrL, uint16_t tar_addrH) {
    this->write_reg(IT8951_LISAR + 2, tar_addrH);
    this->write_reg(IT8951_LISAR, tar_addrL);
}

void IT8951ESensor::write_args(uint16_t cmd, uint16_t *args, uint16_t length) {
    this->write_command(cmd);
    for (uint16_t i = 0; i < length; i++) {
        this->write_word(args[i]);
    }
}

void IT8951ESensor::set_area(uint16_t x, uint16_t y, uint16_t w,
                                  uint16_t h) {
    uint16_t args[5];

    args[0] = (this->m_endian_type << 8 | this->m_pix_bpp << 4);
    args[1] = x;
    args[2] = y;
    args[3] = w;
    args[4] = h;
    this->write_args(IT8951_TCON_LD_IMG_AREA, args, 5);
}

void IT8951ESensor::wait_busy(uint32_t timeout) {
    const uint32_t start_time = millis();
    uint32_t last_wdt_feed = start_time;
    while (true) {
        if (this->busy_pin_->digital_read()) {
            break;
        }

        if (millis() - start_time > timeout) {
            ESP_LOGE(TAG, "Pin busy timeout after %ums", millis() - start_time);
            this->mark_failed();
            break;
        }
        if (millis() - last_wdt_feed > 250) {
            ESP_LOGD(TAG, "Waiting for pin busy, feeding WDT...");
            App.feed_wdt();
            last_wdt_feed = millis();
        }
        delay(2);  // Yield to other tasks
    }
    if (millis() - start_time > 1) {
        ESP_LOGD(TAG, "Busy wait took %dms", millis() - start_time);
    }    
}

void IT8951ESensor::check_busy(uint32_t timeout) {
    const uint32_t start_time = millis();
    uint32_t last_wdt_feed = start_time;
    while (true) {
        this->write_command(IT8951_TCON_REG_RD);
        this->write_word(IT8951_LUTAFSR);
        uint16_t word = this->read_word();
        if (word == 0) {
            break;
        }
 
        if (millis() - start_time > timeout) {
            ESP_LOGE(TAG, "SPI busy timeout %i", word);
            this->mark_failed();
            break;
        }
        if (millis() - last_wdt_feed > 250) {
            ESP_LOGD(TAG, "Waiting for SPI busy, feeding WDT...");
            App.feed_wdt();
            last_wdt_feed = millis();
        }
        delay(2); // Yield to other tasks
    }

    if (millis() - start_time > 1) {
        ESP_LOGD(TAG, "SPI busy wait took %dms", millis() - start_time);
    }    
}

void IT8951ESensor::update_area(uint16_t x, uint16_t y, uint16_t w,
                                     uint16_t h, update_mode_e mode) {
    if (mode == update_mode_e::UPDATE_MODE_NONE) {
        return;
    }

    ESP_LOGV(TAG, "sending update area command");

    // rounded up to be multiple of 4
    x = (x + 3) & 0xFFFC;
    y = (y + 3) & 0xFFFC;

    this->check_busy();

    uint16_t args[7];
    args[0] = x;
    args[1] = y;
    args[2] = w;
    args[3] = h;
    args[4] = mode;
    args[5] = this->IT8951DevAll[this->model_].devInfo.usImgBufAddrL;
    args[6] = this->IT8951DevAll[this->model_].devInfo.usImgBufAddrH;

    this->write_args(IT8951_I80_CMD_DPY_BUF_AREA, args, 7);
}

void IT8951ESensor::reset(void) {
    this->reset_pin_->digital_write(true);
    this->reset_pin_->digital_write(false);
    delay(this->reset_duration_);
    this->reset_pin_->digital_write(true);
    delay(100);
}

uint32_t IT8951ESensor::get_buffer_length_() { return this->get_width_internal() * this->get_height_internal(); }

void IT8951ESensor::get_device_info(struct IT8951DevInfo_s *info) {
    this->write_command(IT8951_I80_CMD_GET_DEV_INFO);
    this->read_words(info, sizeof(struct IT8951DevInfo_s)/2); // Polling HRDY for each words(2-bytes) if possible
}

uint16_t IT8951ESensor::get_vcom() {
    this->write_command(IT8951_I80_CMD_VCOM); // tcon vcom get command
    this->write_word(0x0000);
    const uint16_t vcom = this->read_word();
    ESP_LOGI(TAG, "VCOM = %.02fV", (float)vcom/1000);
    return vcom;
}

void IT8951ESensor::set_vcom(uint16_t vcom) {
    this->write_command(IT8951_I80_CMD_VCOM); // tcon vcom set command
    this->write_word(0x0001);
    this->write_word(vcom);
}

void IT8951ESensor::setup() {
    ESP_LOGCONFIG(TAG, "Init Starting.");
    this->spi_setup();

    if (nullptr != this->reset_pin_) {
        this->reset_pin_->pin_mode(gpio::FLAG_OUTPUT);
        this->reset();
    }

    this->busy_pin_->pin_mode(gpio::FLAG_INPUT | gpio::FLAG_PULLUP);

    /* Not reliable, hard-coded in the model device info (same as M5Stack) */
    //this->get_device_info(&(this->IT8951DevAll[this->model_].devInfo));
    this->dump_config();

    this->write_command(IT8951_TCON_SYS_RUN);

    // enable pack write
    this->write_reg(IT8951_I80CPCR, 0x0001);

    // set vcom to -2.30v
    const uint16_t vcom = this->get_vcom();
    if (2300 != vcom) {
        this->set_vcom(2300);
        this->get_vcom();
    }

    // Allocate display buffer
    this->init_internal_(this->get_buffer_length_());

    ESP_LOGCONFIG(TAG, "Init Done.");
}

/** @brief Write the image at the specified location, Partial update
 * @param x Update X coordinate, >>> Must be a multiple of 4 <<<
 * @param y Update Y coordinate
 * @param w width of gram, >>> Must be a multiple of 4 <<<
 * @param h height of gram
 * @param gram 4bpp gram data
 */
void IT8951ESensor::write_buffer_to_display(uint16_t x, uint16_t y, uint16_t w,
                                            uint16_t h, const uint8_t *gram) {

    ESP_LOGV(TAG, "Writing buffer to display; x=%d, y=%d, w=%d, h=%d", x, y, w, h);
    this->m_endian_type = IT8951_LDIMG_B_ENDIAN;
    this->m_pix_bpp     = IT8951_4BPP;
    if (x > this->get_width() || y > this->get_height()) {
        ESP_LOGE(TAG, "Pos (%d, %d) out of bounds.", x, y);
        return;
    }

    // rounded up to be multiple of 4
    x = (x + 3) & 0xFFFC;
    y = (y + 3) & 0xFFFC;

    this->set_target_memory_addr(this->IT8951DevAll[this->model_].devInfo.usImgBufAddrL, this->IT8951DevAll[this->model_].devInfo.usImgBufAddrH);
    this->set_area(x, y, w, h);

    const uint16_t panel_bytewidth = this->usPanelW_ >> 1; // bytes per row on the panel (2 pixels per byte)
    const uint16_t gram_bytewidth = (w + 1) >> 1; // bytes per row in the gram buffer for the update area

    this->enable();
    /* Send data preamble */
    this->write_byte16(0x0000);

    // Allocate a buffer for one row of pixel data
    ExternalRAMAllocator<uint8_t> allocator(ExternalRAMAllocator<uint8_t>::ALLOW_FAILURE);
    uint8_t *row_buffer = allocator.allocate(gram_bytewidth);
    if (row_buffer == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate row buffer for display update.");
        this->disable();
        return;
    }

    for (uint32_t row = 0; row < h; ++row) {
        uint32_t buf_start_index = (y + row) * panel_bytewidth + (x >> 1);
        memcpy(row_buffer, &gram[buf_start_index], gram_bytewidth);
        if (!this->reversed_) {
            for(uint16_t i = 0; i < gram_bytewidth; i++) row_buffer[i] = 0xFF - row_buffer[i];
        }
        this->transfer_array(row_buffer, gram_bytewidth);
    }

    this->disable();

    this->write_command(IT8951_TCON_LD_IMG_END);
}

void IT8951ESensor::write_display() {
    // If min > max, then there is no update
    if (this->min_x >= this->max_x || this->min_y >= this->max_y) {
        ESP_LOGV(TAG, "No update required, skipping write_display");
        return;
    }

    if (this->sleep_when_done_) {
        this->write_command(IT8951_TCON_SYS_RUN);
    }
    const u_int32_t width = this->max_x - this->min_x + 1;
    const u_int32_t height = this->max_y - this->min_y + 1;
    this->write_buffer_to_display(this->min_x, this->min_y, width, height, this->buffer_);
    this->update_area(this->min_x, this->min_y, width, height, update_mode_e::UPDATE_MODE_A2);   // 2 level
    this->max_x = 0;
    this->max_y = 0;
    this->min_x = this->IT8951DevAll[this->model_].devInfo.usPanelW;
    this->min_y = this->IT8951DevAll[this->model_].devInfo.usPanelH;
    if (this->sleep_when_done_) {
        this->write_command(IT8951_TCON_SLEEP);
    }
}

void IT8951ESensor::write_display_slow() {
    // If min > max, then there is no update
    if (this->min_x >= this->max_x || this->min_y >= this->max_y) {
        ESP_LOGV(TAG, "No update required, skipping write_display_slow");
        return;
    }

    if (this->sleep_when_done_) {
        this->write_command(IT8951_TCON_SYS_RUN);
    }
    const u_int32_t width = this->max_x - this->min_x + 1;
    const u_int32_t height = this->max_y - this->min_y + 1;
    this->write_buffer_to_display(this->min_x, this->min_y, width, height, this->buffer_);
    this->update_area(this->min_x, this->min_y, width, height, update_mode_e::UPDATE_MODE_GC16);
    this->max_x = 0;
    this->max_y = 0;
    this->min_x = this->IT8951DevAll[this->model_].devInfo.usPanelW;
    this->min_y = this->IT8951DevAll[this->model_].devInfo.usPanelH;
    if (this->sleep_when_done_) {
        this->write_command(IT8951_TCON_SLEEP);
    }
}


/** @brief Clear graphics buffer
 * @param init Screen initialization, If is 0, clear the buffer without initializing
 */
void IT8951ESensor::clear(bool init) {
    this->m_endian_type = IT8951_LDIMG_L_ENDIAN;
    this->m_pix_bpp     = IT8951_4BPP;

    ESP_LOGE(TAG, "Clearing display");

    Display::clear();

    this->set_target_memory_addr(this->IT8951DevAll[this->model_].devInfo.usImgBufAddrL, this->IT8951DevAll[this->model_].devInfo.usImgBufAddrH);
    this->set_area(0, 0, this->get_width_internal(), this->get_height_internal());

    const uint16_t bytewidth = this->get_width_internal() >> 1;
    const uint16_t buffer_size = 1024; // Use a 1KB buffer for clearing
    const uint32_t total_bytes = (uint32_t) bytewidth * this->get_height_internal();

    ExternalRAMAllocator<uint8_t> allocator(ExternalRAMAllocator<uint8_t>::ALLOW_FAILURE);
    uint8_t *buffer = allocator.allocate(buffer_size);
    if (buffer == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate buffer for clear().");
        return;
    }
    memset(buffer, 0xFF, buffer_size);

    this->enable();
    /* Send data preamble */
    this->write_byte16(0x0000);

    for (uint32_t i = 0; i < total_bytes; i += buffer_size) {
        uint32_t chunk_size = std::min((uint32_t) buffer_size, total_bytes - i);
        this->transfer_array(buffer, chunk_size);
    }

    allocator.deallocate(buffer, buffer_size);

    this->disable();

    this->write_command(IT8951_TCON_LD_IMG_END);

    if (init) {
        this->update_area(0, 0, this->get_width_internal(), this->get_height_internal(), update_mode_e::UPDATE_MODE_INIT);
    }
}

void IT8951ESensor::update() {

    ESP_LOGV(TAG, "update called");
    if (this->is_ready()) {
        this->do_update_();
        this->write_display();
    }
}

void IT8951ESensor::update_slow() {
    ESP_LOGV(TAG, "update_slow called");
    if (this->is_ready()) {
        this->do_update_();
        this->write_display_slow();
    }
}

void IT8951ESensor::fill(Color color) {
    for (uint32_t i = 0; i < this->get_buffer_length_(); i++) {
        this->buffer_[i] = 0x00;
    }
    this->max_x = this->get_width_internal() - 1;
    this->max_y = this->get_height_internal() - 1;
    this->min_x = 0;
    this->min_y = 0;
}

void HOT IT8951ESensor::draw_pixel_at(int x, int y, Color color) {
    // This is called by the base class with pre-rotated coordinates.
    display::DisplayBuffer::draw_pixel_at(x, y, color);
}

void IT8951ESensor::draw_pixels_at(int x_start, int y_start, int w, int h, const uint8_t *ptr,
                                   display::ColorOrder order, display::ColorBitness bitness, bool big_endian,
                                   int x_offset, int y_offset, int x_pad) {
    if (bitness != display::COLOR_BITNESS_565) {
        // Fallback to slow method if format is not what we expect from LVGL

        ESP_LOGE(TAG, "Falling back to slow drawing method, as color bitness is not 565.");
        display::DisplayBuffer::draw_pixels_at(x_start, y_start, w, h, ptr, order, bitness, big_endian, x_offset,
                                               y_offset, x_pad);
        return;
    }

    // Update dirty area
    if (this->min_x > x_start) this->min_x = x_start;
    if (this->min_y > y_start) this->min_y = y_start;
    if (this->max_x < x_start + w - 1) this->max_x = x_start + w - 1;
    if (this->max_y < y_start + h - 1) this->max_y = y_start + h - 1;

    const uint16_t panel_bytewidth = this->usPanelW_ >> 1;
    size_t line_stride = x_offset + w + x_pad;  // length of each source line in pixels

    for (int y = 0; y < h; y++) {
        int dst_y = y_start + y;
        const uint16_t *src_addr = (const uint16_t *)(ptr + (((y_offset + y) * line_stride) + x_offset) * 2);
        uint8_t *dst_addr = this->buffer_ + (dst_y * panel_bytewidth) + (x_start / 2);

        for (int x = 0; x < w; x += 2) {
            // Process pixel 1 (even)
            uint16_t color16_1 = src_addr[x];
            if (big_endian) {
                color16_1 = __builtin_bswap16(color16_1);
            }
            uint8_t r1 = (color16_1 & 0xF800) >> 11;
            uint8_t g1 = (color16_1 & 0x07E0) >> 5;
            uint8_t b1 = (color16_1 & 0x001F);
            // Convert to 4-bit grayscale, then threshold to black (0) or white (15)
            uint8_t gray4_1 = ((r1 * 2126 + g1 * 7152 + b1 * 722) >> 12) > 7 ? 0xF : 0x0;

            uint8_t gray4_2 = gray4_1; // Default for odd width
            if (x + 1 < w) {
                // Process pixel 2 (odd)
                uint16_t color16_2 = src_addr[x + 1];
                if (big_endian) {
                    color16_2 = __builtin_bswap16(color16_2);
                }
                uint8_t r2 = (color16_2 & 0xF800) >> 11;
                uint8_t g2 = (color16_2 & 0x07E0) >> 5;
                uint8_t b2 = (color16_2 & 0x001F);
                gray4_2 = ((r2 * 2126 + g2 * 7152 + b2 * 722) >> 12) > 7 ? 0xF : 0x0;
            }

            // Combine two 4-bit pixels into one byte and write
            *dst_addr = (gray4_1 << 4) | gray4_2;
            dst_addr++;
        }
    }
    App.feed_wdt();
}

void HOT IT8951ESensor::draw_absolute_pixel_internal(int x, int y, Color color) {
    // Fast path: bounds and buffer check first
    if (x < 0 || y < 0 || this->buffer_ == nullptr || x >= this->usPanelW_ || y >= this->usPanelH_) {
        return;
    }

    // Track min/max for partial updates
    if (x > this->max_x) {
        this->max_x = x;
    }
    if (y > this->max_y) {
        this->max_y = y;
    }
    if (x < this->min_x) {
        this->min_x = x;
    }
    if (y < this->min_y) {
        this->min_y = y;
    }

    uint32_t internal_color = color.raw_32 & 0x0F;
    uint16_t _bytewidth = this->usPanelW_ >> 1;

    uint32_t index = static_cast<uint32_t>(y) * _bytewidth + (static_cast<uint32_t>(x) >> 1);

    uint8_t &buf = this->buffer_[index];
    if (x & 0x1) {
        // Odd pixel: lower nibble
        buf = (buf & 0xF0) | internal_color;
    } else {
        // Even pixel: upper nibble
        buf = (buf & 0x0F) | (internal_color << 4);
    }
}

void IT8951ESensor::set_model(it8951eModel model) {
    this->model_ = model;
    // Provide fast access to panel width and height
    usPanelW_ = IT8951DevAll[model].devInfo.usPanelW;
    usPanelH_ = IT8951DevAll[model].devInfo.usPanelH;
}

void IT8951ESensor::dump_config() {
    LOG_DISPLAY("", "IT8951E", this);
    switch (this->model_) {
    case it8951eModel::M5EPD:
        ESP_LOGCONFIG(TAG, "  Model: M5EPD");
        break;
    default:
        ESP_LOGCONFIG(TAG, "  Model: unkown");
        break;
    }
    ESP_LOGCONFIG(TAG, "LUT: %s, FW: %s, Mem:%x (%d x %d)",
        this->IT8951DevAll[this->model_].devInfo.usLUTVersion,
        this->IT8951DevAll[this->model_].devInfo.usFWVersion,
        this->IT8951DevAll[this->model_].devInfo.usImgBufAddrL | (this->IT8951DevAll[this->model_].devInfo.usImgBufAddrH << 16),
        this->IT8951DevAll[this->model_].devInfo.usPanelW,
        this->IT8951DevAll[this->model_].devInfo.usPanelH
    );
}

}  // namespace it8951e
}  // namespace esphome
