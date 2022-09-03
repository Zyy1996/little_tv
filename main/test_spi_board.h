#include "sdkconfig.h"

#define TEST_LCD_H_RES          (240)
#define TEST_LCD_V_RES          (240)

#define TEST_LCD_BK_LIGHT_GPIO  (18)
#define TEST_LCD_RST_GPIO       (5)
#define TEST_LCD_CS_GPIO        (0)
#define TEST_LCD_DC_GPIO        (19)
#define TEST_LCD_PCLK_GPIO      (2)
#define TEST_LCD_DATA0_GPIO     (4)
#define TEST_LCD_DATA1_GPIO     (7)
#define TEST_LCD_DATA2_GPIO     (8)
#define TEST_LCD_DATA3_GPIO     (9)
#define TEST_LCD_DATA4_GPIO     (10)
#define TEST_LCD_DATA5_GPIO     (11)
#define TEST_LCD_DATA6_GPIO     (12)
#define TEST_LCD_DATA7_GPIO     (13)

void lcd_initialize_spi(esp_lcd_panel_io_handle_t *io_handle, esp_lcd_panel_io_color_trans_done_cb_t on_color_trans_done, void *user_ctx, int cmd_bits, int param_bits, bool oct_mode);
void lcd_panel_test(esp_lcd_panel_io_handle_t io_handle, esp_lcd_panel_handle_t panel_handle);