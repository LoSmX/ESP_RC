
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "time.h"
#include "sys/time.h"

#define SPP_TAG "BT_RC"
#define EXCAMPLE_DEVICE_NAME "XMC-Bluetooth"
#define SPP_DATA_LEN 16

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_NONE;
static const esp_spp_role_t role_master = ESP_SPP_ROLE_SLAVE;

static esp_bd_addr_t peer_bd_addr;
static uint8_t peer_bdname_len;
static char peer_bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];
static const char remote_device_name[] = "XMC-Bluetooth";
static const esp_bt_inq_mode_t inq_mode = ESP_BT_INQ_MODE_GENERAL_INQUIRY;
static const uint8_t inq_len = 30;
static const uint8_t inq_num_rsps = 0;

static uint8_t spp_data[SPP_DATA_LEN];

static int con_state = 1;
static uint32_t my_hand = 0;
/***************************************************************************/

void set_roll_in_data( uint16_t value )
{
    spp_data[5] = (uint8_t) value;
    spp_data[6] = (uint8_t) (value >> 8);
}

void set_pitch_in_data( uint16_t value )
{
    spp_data[7] = (uint8_t) value;
    spp_data[8] = (uint8_t) (value >> 8);
}

void set_throttle_in_data( uint16_t value )
{
    spp_data[9] = (uint8_t) value;
    spp_data[10] = (uint8_t) (value >> 8);
}

void set_yaw_in_data( uint16_t value )
{
    spp_data[11] = (uint8_t) value;
    spp_data[12] = (uint8_t) (value >> 8);
}

void set_arm_in_data( uint16_t value )
{
    spp_data[13] = (uint8_t) value;
    spp_data[14] = (uint8_t) (value >> 8);
}

void set_crc_in_data( void )
{   
    uint8_t crc = 0;
    for(int i = 3; i < 15; i++){
        crc = crc ^ spp_data[i];
    }
    spp_data[15] = crc;
}

static bool get_name_from_eir(uint8_t *eir, char *bdname, uint8_t *bdname_len)
{
    uint8_t *rmt_bdname = NULL;
    uint8_t rmt_bdname_len = 0;

    if (!eir) {
        return false;
    }

    rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
    if (!rmt_bdname) {
        rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
    }

    if (rmt_bdname) {
        if (rmt_bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN) {
            rmt_bdname_len = ESP_BT_GAP_MAX_BDNAME_LEN;
        }

        if (bdname) {
            memcpy(bdname, rmt_bdname, rmt_bdname_len);
            bdname[rmt_bdname_len] = '\0';
        }
        if (bdname_len) {
            *bdname_len = rmt_bdname_len;
        }
        return true;
    }

    return false;
}

// Serial port profile callback
static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{   
    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
        esp_bt_dev_set_device_name(EXCAMPLE_DEVICE_NAME);
        esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
        esp_bt_gap_start_discovery(inq_mode, inq_len, inq_num_rsps);

        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT status=%d scn_num=%d",param->disc_comp.status, param->disc_comp.scn_num);
        if (param->disc_comp.status == ESP_SPP_SUCCESS) {
            esp_spp_connect(sec_mask, role_master, param->disc_comp.scn[0], peer_bd_addr);
        }
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
        con_state = 0;
        my_hand = param->open.handle;
        
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT");
        break;
    case ESP_SPP_START_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT");
        printf("Recived: %s\n ", param->data_ind.data);
        break;
    case ESP_SPP_CONG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT cong=%d", param->cong.cong);

        if (param->cong.cong == 0) {
            /* if(esp_spp_write(param->cong.handle, SPP_DATA_LEN, spp_data)!= ESP_OK){
                printf("ERROR: send failed !!! \n");
            } */
        }
        break;
    case ESP_SPP_WRITE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT len=%d cong=%d", param->write.len , param->write.cong);
        esp_log_buffer_hex("",spp_data,SPP_DATA_LEN);
        if (param->write.cong == 0) {
            /*if(esp_spp_write(param->write.handle, SPP_DATA_LEN, spp_data)!= ESP_OK){
                printf("ERROR: send failed !!! \n");
            }*/
        }
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");
        break;
    default:
        break;
    }
}

//GAP callback search for drone and conects t it
static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch(event){
    case ESP_BT_GAP_DISC_RES_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_DISC_RES_EVT");
        esp_log_buffer_hex(SPP_TAG, param->disc_res.bda, ESP_BD_ADDR_LEN);
        for (int i = 0; i < param->disc_res.num_prop; i++){
            //If drone found start spp dicovery and cancle gap discovery
            if (param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_EIR
                && get_name_from_eir(param->disc_res.prop[i].val, peer_bdname, &peer_bdname_len)){
                esp_log_buffer_char(SPP_TAG, peer_bdname, peer_bdname_len);
                if (strlen(remote_device_name) == peer_bdname_len
                    && strncmp(peer_bdname, remote_device_name, peer_bdname_len) == 0) {
                    memcpy(peer_bd_addr, param->disc_res.bda, ESP_BD_ADDR_LEN);
                    esp_spp_start_discovery(peer_bd_addr);
                    esp_bt_gap_cancel_discovery();
                }
            }
        }
        break;
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_DISC_STATE_CHANGED_EVT");
        break;
    case ESP_BT_GAP_RMT_SRVCS_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_RMT_SRVCS_EVT");
        break;
    case ESP_BT_GAP_RMT_SRVC_REC_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_RMT_SRVC_REC_EVT");
        break;
    case ESP_BT_GAP_AUTH_CMPL_EVT:{
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(SPP_TAG, "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    default:
        break;
    }
}

//Initialisation of bluetoot gat and spp
void app_main()
{
    //setup default spp_data

    //Preamble
    spp_data[0] = '$';
    spp_data[1] = 'M';
    spp_data[2] = '<';
        
    //Size and Type
    spp_data[3] = 10;
    spp_data[4] = 200;

    //payload and crc
    set_roll_in_data(1500);
    set_pitch_in_data(1500);
    set_throttle_in_data(1000);
    set_yaw_in_data(1500);
    set_arm_in_data(1000);
    set_crc_in_data();

    // initializing non volatile storage
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }


    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);
    
    while(con_state){

    }
    int j = 0;
    while (1)
    {
         
        for(int i = 0 ; i < 1000000; i++){
            if(i == 1){    
                if(esp_spp_write(my_hand, SPP_DATA_LEN, spp_data) != ESP_OK){
                    printf("ERROR: send failed !!! \n");
                }else{
                    printf("Send\n");
                }
                j++;
            }
        }
        if(j == 100){ 
            set_arm_in_data(2000);
            set_crc_in_data();
        }
    }
    
}