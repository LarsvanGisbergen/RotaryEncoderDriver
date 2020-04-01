

#include "rotaryEncoder.h"
#include "driver/i2c.h"
#include "esp_log.h"

/*READ FIRST: INFORMATION ABOUT REGISTRY VALUES, MAXIMUM AND MIMIMUMS, ETC.
  IS GIVEN IN THE HEADER OF THIS PROJECT.*/

// disable buffers
static const size_t I2C_MASTER_TX_BUF_DISABLE = 0;
static const size_t I2C_MASTER_RX_BUF_DISABLE = 0;
static const int INTR_FLAGS = 0;

static const char *TAG = "I2C_RotaryEncoder";
i2c_port_t port;


/*checks if a connection exists for the rotary encoder*/
bool RotaryEncoder_isConnected()
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, RE_I2C_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    if (i2c_master_stop(cmd) != 0)
        return (false);
    i2c_cmd_link_delete(cmd);
    return (true);
}

/*Initializes the rotary encoder by making a configuration and implementing it*/
RE_err_t RotaryEncoder_init()
{
    esp_err_t ret;
    // setup i2c controller
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 18,
        .scl_io_num = 23,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 20000};
    ret = i2c_param_config(port, &conf);
    i2c_set_timeout(I2C_NUM_0, 20000);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "PARAM CONFIG FAILED");
        return RE_ERR_OK;
    }
    ESP_LOGV(TAG, "PARAM CONFIG done");

    // install the driver
    ret = i2c_driver_install(port, I2C_MODE_MASTER, I2C_MASTER_TX_BUF_DISABLE, I2C_MASTER_RX_BUF_DISABLE, INTR_FLAGS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C driver install failed or already installed!");
        return RE_ERR_INSTALL;
    }
    ESP_LOGV(TAG, "I2C DRIVER INSTALLED");

    return RE_ERR_OK;
}
/*writes an 8bit unsigned integer value to a register*/
RE_err_t RotaryEncoder_write_register(RE_reg_t reg, uint8_t v)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, RE_I2C_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, v, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL)
    {
        ESP_LOGE(TAG, "ERROR: unable to write to register");
        return RE_ERR_FAIL;
    }
    return RE_ERR_OK;
}

/*writes a 16bit unsigned integer value to a register*/
RE_err_t RotaryEncoder_write_register16(RE_reg_t reg_LSB, uint16_t v)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (RE_I2C_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_LSB, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, v & 0xff, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, v >> 8, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL)
    {
        return RE_ERR_FAIL;
    }
    return RE_ERR_OK;
}
/*reads an 8bit unsigned integer value from a register*/
RE_err_t RotaryEncoder_read_register(RE_reg_t reg, uint8_t *data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (RE_I2C_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL)
    {
        return RE_ERR_FAIL;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (RE_I2C_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data, 1); //data received from the register is set to the var: data.
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL)
    {
        return RE_ERR_FAIL;
    }

    ESP_LOGI(TAG, "received: %d", *data);

    return RE_ERR_OK;
}


/*reads an 16bit unsigned integer value from a register*/
RE_err_t RotaryEncoder_read_register16(RE_reg_t reg_LSB, uint16_t *data)
{
    uint8_t LSB = 0x00;
    uint8_t MSB = 0x00;
	//Reading from i2c_master only gives an 8bit integer meaning we will have to make this call twice to make two 8bit integers into a 16bit integer.
    //MSB
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (RE_I2C_ADDR << 1), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_LSB + 1, ACK_CHECK_EN); 
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (RE_I2C_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &MSB, 1); //first 8bit integer is set to most significant bit.
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL)
    {
        return RE_ERR_FAIL;
    }
    //LSB
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (RE_I2C_ADDR << 1), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_LSB, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (RE_I2C_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &LSB, 1); //the second read is to get the least significant bit
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL)
    {
        return RE_ERR_FAIL;
    }
    uint16_t temp = ((uint16_t)MSB << 8 | LSB); //the most significant and least significant bit are used to create our 16bit integer.

    // ESP_LOGI(TAG, "data: %d", temp);
    *data = temp; //the genererated 16bit integer is saved into var: data.
    return ret;
}

/*Reads the current status from the STAT register. (specifics about status are found in the header)*/
RE_err_t RotaryEncoder_getStatus(uint8_t *data)
{
    RotaryEncoder_read_register(STAT, &(*data));
    return RE_ERR_OK;
}

/*Sets the availability of the interrupt. (specifics about the interupt workings are found in the header)*/
RE_err_t RotaryEncoder_interruptEnable(bool v)
{
    v ? RotaryEncoder_write_register(STAT, 1) : RotaryEncoder_write_register(STAT, 0);
    return RE_ERR_OK;
}
/*Sets the brightness of a specific color to the desired 8bit value*/
RE_err_t RotaryEncoder_setColorRed(uint8_t v)
{
    esp_err_t ret = RotaryEncoder_write_register(LED_BRIGHTNESS_RED, v);
    if (ret == ESP_FAIL)
    {
        return RE_ERR_FAIL;
    }
    return RE_ERR_OK;
}
/*Sets the brightness of a specific color to the desired 8bit value*/
RE_err_t RotaryEncoder_setColorGrn(uint8_t v)
{
    esp_err_t ret = RotaryEncoder_write_register(LED_BRIGHTNESS_GRN, v);
    if (ret == ESP_FAIL)
    {
        return RE_ERR_FAIL;
    }
    return RE_ERR_OK;
}
/*Sets the brightness of a specific color to the desired 8bit value*/
RE_err_t RotaryEncoder_setColorBlu(uint8_t v)
{
    esp_err_t ret = RotaryEncoder_write_register(LED_BRIGHTNESS_BLU, v);
    if (ret == ESP_FAIL)
    {
        return RE_ERR_FAIL;
    }
    return RE_ERR_OK;
}
/*Sets the amount the brightness is changed with each tick of the encoder for a specific color*/
RE_err_t RotaryEncoder_connectColorRed(uint16_t v)
{
    esp_err_t ret = RotaryEncoder_write_register16(LED_CON_RED_LSB, v);
    if (ret == ESP_FAIL)
    {
        return RE_ERR_FAIL;
    }
    return RE_ERR_OK;
}

/*Sets the amount the brightness is changed with each tick of the encoder for a specific color*/
RE_err_t RotaryEncoder_connectColorGrn(uint16_t v)
{
    esp_err_t ret = RotaryEncoder_write_register16(LED_CON_GRN_LSB, v);
    if (ret == ESP_FAIL)
    {
        return RE_ERR_FAIL;
    }
    return RE_ERR_OK;
}

/*Sets the amount the brightness is changed with each tick of the encoder for a specific color*/
RE_err_t RotaryEncoder_connectColorBlu(uint16_t v)
{
    esp_err_t ret = RotaryEncoder_write_register16(LED_CON_BLU_LSB, v);
    if (ret == ESP_FAIL)
    {
        return RE_ERR_FAIL;
    }
    return RE_ERR_OK;
}
/*Gets the amount of ticks the user has twisted the knob*/
RE_err_t RotaryEncoder_getCount(uint16_t *data)
{
    esp_err_t ret = RotaryEncoder_read_register16(RE_COUNT_LSB, &(*data));
    if (ret == ESP_FAIL)
    {
        return RE_ERR_FAIL;
    }
    return RE_ERR_OK;
}
/*Gets the amount of ticks the user has twisted the knob since last measure*/
RE_err_t RotaryEncoder_getDiff(uint16_t *data)
{

    esp_err_t ret = RotaryEncoder_read_register16(RE_DIFF_LSB, &(*data));
    if (ret == ESP_FAIL)
    {
        return RE_ERR_FAIL;
    }
    ret = RotaryEncoder_write_register16(RE_DIFF_LSB, 0x00);
    if (ret == ESP_FAIL)
    {
        return RE_ERR_FAIL;
    }
    return RE_ERR_OK;
}
/*Gets the time in milliseconds since last measured movement (of encoder)*/
RE_err_t RotaryEncoder_getTSLM(uint16_t *data)
{
    esp_err_t ret = RotaryEncoder_read_register16(RE_TSLM_LSB, &(*data));
    if (ret == ESP_FAIL)
    {
        return RE_ERR_FAIL;
    }
    ret = RotaryEncoder_write_register16(RE_TSLB_LSB, 0x00);
    if (ret == ESP_FAIL)
    {
        return RE_ERR_FAIL;
    }
    return RE_ERR_OK;
}

/*Gets the time in milliseconds since last measured press/release event (of encoder)*/
RE_err_t RotaryEncoder_getTSLB(uint16_t *data)
{
    esp_err_t ret = RotaryEncoder_read_register16(RE_DIFF_LSB, &(*data));
    if (ret == ESP_FAIL)
    {
        return RE_ERR_FAIL;
    }
    ret = RotaryEncoder_write_register16(RE_DIFF_LSB, 0x00);
    if (ret == ESP_FAIL)
    {
        return RE_ERR_FAIL;
    }
    return RE_ERR_OK;
}
/*Left empty intentionally. Has no function yet*/
RE_err_t RotaryEncoder_set_bit(uint8_t bit, RE_reg_t reg)
{
    // empty
    return RE_ERR_OK;
}
/*Left empty intentionally. Has no function yet*/
RE_err_t RotaryEncoder_clear_bit(uint8_t bit, RE_reg_t reg)
{
    // empty
    return RE_ERR_OK;
}