#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/string.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "driver/twai.h"

#define TX_GPIO_NUM                     GPIO_NUM_21
#define RX_GPIO_NUM                     GPIO_NUM_22
#define TAG                     		"TWAI Controller"

#define ID_ESP32                        0x001
#define SDO_ID_DRIVER_1                 0x603
#define SDO_ID_DRIVER_2                 0x602
#define NMT_START_STOP_ID               0x0
#define R_PDO1_ID_DRIVER_1              0x203
#define R_PDO1_ID_DRIVER_2              0x202
#define R_PDO2_ID_DRIVER_1              0x303
#define R_PDO2_ID_DRIVER_2              0x302

// Definiciones de pines

#define GPIO_DIR_CONTROL GPIO_NUM_25   // Pin para salida digital 1 o 0
#define PWM_OUTPUT_PIN GPIO_NUM_27    // Pin para salida PWM
#define GPIO_DIR_CONTROL2 GPIO_NUM_33   // Pin para salida digital 1 o 0
#define PWM_OUTPUT_PIN2 GPIO_NUM_26    // Pin para salida PWM
#define SENSOR GPIO_NUM_14    // Pin para sensor de proximidad
#define SENSOR_IR_B1 GPIO_NUM_19
#define SENSOR_IR_B2 GPIO_NUM_18

// Definiciones de PWM
#define PWM_FREQUENCY 5000   // Frecuencia de 5 kHz
#define PWM_FREQUENCY2 1000   // Frecuencia de 1 kHz
#define PWM_DUTY_CYCLE 50    // Ciclo de trabajo al 50%

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_subscription_t motor_angle_subscriber1;
rcl_subscription_t motor_angle_subscriber2;
rcl_publisher_t esp32_publisher;
geometry_msgs__msg__Vector3 motor_angle1;
geometry_msgs__msg__Vector3 motor_angle2;
std_msgs__msg__String msg;

uint8_t positive_end[8] = {0x2B,0x06,0x30,0x10,0x02,0x00,0x00,0x00};
uint8_t negative_end[8] = {0x2B,0x06,0x30,0x0F,0x02,0x00,0x00,0x00};
uint8_t activate_rpdo2_d1[8] = {0x23,0x01,0x14,0x01,0x03,0x03,0x00,0x04};
uint8_t activate_rpdo2_d2[8] = {0x23,0x01,0x14,0x01,0x02,0x03,0x00,0x04};
uint8_t set_accel[8] = {0x23,0x83,0x60,0x00,0xD0,0x07,0x00,0x00};
uint8_t set_decel[8] = {0x23,0x84,0x60,0x00,0xA0,0x0F,0x00,0x00};
uint8_t target_vel[8] = {0x23, 0x81, 0x60, 0x00, 0xA0, 0x0F, 0x00, 0x00};
uint8_t nmt_start[2] = {0x01,0x00};
uint8_t rpdo2_1[6] = {0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t rpdo2_2[6] = {0x06,0x00,0x00,0x00,0x00,0x00};
uint8_t rpdo2_3[6] = {0x0F,0x00,0x00,0x00,0x00,0x00};
uint8_t start_op_mode_pos[8] = {0x2F,0x60,0x60,0x00,0x01,0x00,0x00,0x00};
uint8_t velocity_search_limit[8] = {0x23,0x99,0x60,0x01,0xBC,0x02,0x00,0x00};
uint8_t velocity_move_away[8] = {0x23,0x99,0x60,0x02,0x64,0x00,0x00,0x00};
uint8_t rpdo1_1[2] = {0x00,0x00};
uint8_t rpdo1_2[2] = {0x06,0x00};
uint8_t rpdo1_3[2] = {0x0F,0x00};
uint8_t start_op_mode_home[8] = {0x2F,0x60,0x60,0x00,0x06,0x00,0x00,0x00};
uint8_t select_method_home[8] = {0x2F,0x98,0x60,0x00,0x11,0x00,0x00,0x00};
uint8_t start_home[2] = {0x1F,0x00};
uint8_t scale_pos_den[8] = {0x43,0x06,0x30,0x07,0x10,0x0E,0x00,0x00};
uint8_t scale_pos_den_read[8] = {0x40,0x06,0x30,0x07,0x00,0x00,0x00,0x00};

union IntToBytes {
    int num;
    unsigned char bytes[4];
};

union IntToBytes m1_ang;
union IntToBytes m2_ang;
int conv_rel_paces_angle1 = 3600; // paces/1 grade
int conv_rel_paces_angle2 = 4480; // paces/1 grade
float conv_rel_paces_angle3 = 1.42;
int conv_rel_paces_cm = 793; // paces/1 cm
float pos_m4 = 25.4; //cm
float pos_m1 = 0; //grados
float pos_m2 = 0; //grados
float pos_m3 = 0; //grados

static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_50KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

static twai_message_t sdo_message = {
    // Message type and format settings
    .extd = 0,                // Standard Format message (11-bit ID)
    .rtr = 0,                 // Send a data frame
    .ss = 0,                  // Not single shot
    .self = 0,                // Not a self reception request
    .dlc_non_comp = 0,        // DLC is less than 8
    // Message ID and payload
    .identifier = SDO_ID_DRIVER_1,   // COB-ID, funcion + id del nodo al que va dirigido el mensaje
    .data_length_code = 8,
    .data = {1, 2, 3, 4, 5, 6, 7, 8},
};

static twai_message_t r_pdo2_message = {
    // Message type and format settings
    .extd = 0,                // Standard Format message (11-bit ID)
    .rtr = 0,                 // Send a data frame
    .ss = 0,                  // Not single shot
    .self = 0,                // Not a self reception request
    .dlc_non_comp = 0,        // DLC is less than 8
    // Message ID and payload
    .identifier = R_PDO2_ID_DRIVER_1,   // COB-ID, funcion + id del nodo al que va dirigido el mensaje
    .data_length_code = 6,
    .data = {1, 2, 3, 4, 5, 6},
};

static twai_message_t r_pdo1_message = {
    // Message type and format settings
    .extd = 0,                // Standard Format message (11-bit ID)
    .rtr = 0,                 // Send a data frame
    .ss = 0,                  // Not single shot
    .self = 0,                // Not a self reception request
    .dlc_non_comp = 0,        // DLC is less than 8
    // Message ID and payload
    .identifier = R_PDO1_ID_DRIVER_1,   // COB-ID, funcion + id del nodo al que va dirigido el mensaje
    .data_length_code = 2,
    .data = {1, 2},
};

static twai_message_t nmt_ss_message = {
    // Message type and format settings
    .extd = 0,                // Standard Format message (11-bit ID)
    .rtr = 0,                 // Send a data frame
    .ss = 0,                  // Not single shot
    .self = 0,                // Not a self reception request
    .dlc_non_comp = 0,        // DLC is less than 8
    // Message ID and payload
    .identifier = NMT_START_STOP_ID,   // COB-ID, funcion + id del nodo al que va dirigido el mensaje
    .data_length_code = 2,
    .data = {1, 2},
};

void send_can_message(twai_message_t type_message, int identifier, uint8_t* message, bool receive)
{
    type_message.identifier = identifier;
    for (int i = 0; i < type_message.data_length_code; i++) {
        type_message.data[i] = message[i];
    }
    twai_transmit(&type_message, portMAX_DELAY);
    if (receive){
		twai_message_t rx_msg;
		twai_receive(&rx_msg, portMAX_DELAY);
		uint64_t data = 0;
		if (rx_msg.identifier != ID_ESP32){
			for (int i = 0; i < rx_msg.data_length_code; i++) {
				data |= ((uint64_t)rx_msg.data[i] << (i * 8));
				ESP_LOGI(TAG, "Received byte value %02X", rx_msg.data[i]);
			}
			ESP_LOGI(TAG, "Received data value %"PRIu64, data);
		}
    }
}

void move_motor_can(int driver, union IntToBytes angle)
{
	uint8_t start_move[6] = {0x5F,0x00,angle.bytes[0],angle.bytes[1],angle.bytes[2],angle.bytes[3]};
	uint8_t new_setpoint[6] = {0x4F,0x00,angle.bytes[0],angle.bytes[1],angle.bytes[2],angle.bytes[3]};
	if (driver == 1){
		send_can_message(sdo_message, SDO_ID_DRIVER_1, activate_rpdo2_d1, 1);
		send_can_message(sdo_message, SDO_ID_DRIVER_1, set_accel, 1);
		send_can_message(sdo_message, SDO_ID_DRIVER_1, set_decel, 1);
		send_can_message(sdo_message, SDO_ID_DRIVER_1, target_vel, 1);
		send_can_message(nmt_ss_message, NMT_START_STOP_ID, nmt_start, 0);
		send_can_message(r_pdo2_message, R_PDO2_ID_DRIVER_1, rpdo2_1, 0);
		send_can_message(sdo_message, SDO_ID_DRIVER_1, negative_end, 1);
		send_can_message(sdo_message, SDO_ID_DRIVER_1, positive_end, 1);
		send_can_message(r_pdo2_message, R_PDO2_ID_DRIVER_1, rpdo2_2, 0);
		send_can_message(r_pdo2_message, R_PDO2_ID_DRIVER_1, rpdo2_3, 1);
		send_can_message(sdo_message, SDO_ID_DRIVER_1, start_op_mode_pos, 1);
		send_can_message(r_pdo2_message, R_PDO2_ID_DRIVER_1, new_setpoint, 1);
		send_can_message(r_pdo2_message, R_PDO2_ID_DRIVER_1, start_move, 1);
	}
	else if (driver == 2)
	{
		send_can_message(sdo_message, SDO_ID_DRIVER_2, activate_rpdo2_d2, 1);
		send_can_message(sdo_message, SDO_ID_DRIVER_2, set_accel, 1);
		send_can_message(sdo_message, SDO_ID_DRIVER_2, set_decel, 1);
		send_can_message(sdo_message, SDO_ID_DRIVER_2, target_vel, 1);
		send_can_message(nmt_ss_message, NMT_START_STOP_ID, nmt_start, 0);
		send_can_message(r_pdo2_message, R_PDO2_ID_DRIVER_2, rpdo2_1, 0);
		send_can_message(sdo_message, SDO_ID_DRIVER_2, positive_end, 1);
		send_can_message(sdo_message, SDO_ID_DRIVER_2, negative_end, 1);
		send_can_message(r_pdo2_message, R_PDO2_ID_DRIVER_2, rpdo2_2, 0);
		send_can_message(r_pdo2_message, R_PDO2_ID_DRIVER_2, rpdo2_3, 1);
		send_can_message(sdo_message, SDO_ID_DRIVER_2, start_op_mode_pos, 1);
		send_can_message(r_pdo2_message, R_PDO2_ID_DRIVER_2, new_setpoint, 1);
		send_can_message(r_pdo2_message, R_PDO2_ID_DRIVER_2, start_move, 1);
	}
	else
	{
		ESP_LOGI(TAG, "Incorrect driver, must use 1 or 2");
	}
}

void can_home_right_b1(){
    while(1){
        int level_b1 = gpio_get_level(SENSOR_IR_B1);
        printf("Sensor b1: %d", level_b1);
        if (level_b1 == 0){
            m1_ang.num = 0;
            break;
        }
        else {
            ESP_ERROR_CHECK(twai_start());
            ESP_LOGI(TAG, "Driver started");

            m1_ang.num = 1000;
            move_motor_can(1, m1_ang);

            //Wait for bus to become free
            twai_status_info_t status_info;
            twai_get_status_info(&status_info);
            while (status_info.msgs_to_tx > 0) {
                vTaskDelay(pdMS_TO_TICKS(100));
                twai_get_status_info(&status_info);
            }
            ESP_ERROR_CHECK(twai_stop());
            ESP_LOGI(TAG, "Driver stopped");
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void can_home_right_b2(){
    while(1){
        int level_b2 = gpio_get_level(SENSOR_IR_B2);
        printf("Sensor b2: %d", level_b2);
        if (level_b2 == 0){
            m2_ang.num = 0;
            break;
        }
        else {
            ESP_ERROR_CHECK(twai_start());
            ESP_LOGI(TAG, "Driver started");

            m2_ang.num = 1000;
            move_motor_can(2, m2_ang);

            //Wait for bus to become free
            twai_status_info_t status_info;
            twai_get_status_info(&status_info);
            while (status_info.msgs_to_tx > 0) {
                vTaskDelay(pdMS_TO_TICKS(100));
                twai_get_status_info(&status_info);
            }
            ESP_ERROR_CHECK(twai_stop());
            ESP_LOGI(TAG, "Driver stopped");
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void can_home(){
    can_home_right_b1();
    can_home_right_b2();
    pos_m1 = 0;
    pos_m2 = 0;
}

void init_gpio(void) {
    gpio_config_t io_conf_output = {};
    io_conf_output.intr_type = GPIO_INTR_DISABLE;
    io_conf_output.mode = GPIO_MODE_OUTPUT;
    io_conf_output.pin_bit_mask = (1ULL<<GPIO_DIR_CONTROL) | (1ULL<<GPIO_DIR_CONTROL2);
    io_conf_output.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf_output.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf_output);

    // Configura el pin como entrada
    gpio_config_t io_conf_input = {};
    io_conf_input.intr_type = GPIO_INTR_DISABLE;
    io_conf_input.mode = GPIO_MODE_INPUT;
    io_conf_input.pin_bit_mask = (1ULL << SENSOR) | (1ULL << SENSOR_IR_B1) | (1ULL << SENSOR_IR_B2); 
    io_conf_input.pull_down_en = 0;
    io_conf_input.pull_up_en = 0; 
    gpio_config(&io_conf_input);
}

void init_pwm(void) {
    // Configuración del PWM
    ledc_timer_config_t pwm_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&pwm_timer);

    // Configuración del PWM
    ledc_timer_config_t pwm_timer2 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = PWM_FREQUENCY2,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&pwm_timer2);

    // Canal del PWM
    ledc_channel_config_t pwm_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PWM_OUTPUT_PIN,
        .duty = 0, // Ciclo de trabajo inicial
        .hpoint = 0
    };
    ledc_channel_config(&pwm_channel);

    // Canal PWM 2
    ledc_channel_config_t pwm_channel_2 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,         // Canal 1
        .timer_sel = LEDC_TIMER_1,         // Utiliza el mismo temporizador (0)
        .intr_type = LEDC_INTR_DISABLE,    // Sin interrupciones
        .gpio_num = PWM_OUTPUT_PIN2,      // Segundo pin PWM
        .duty = 0,                         // Ciclo de trabajo inicial (0%)
        .hpoint = 0
    };
    ledc_channel_config(&pwm_channel_2);
}

void set_pwm_duty_cycle(int duty) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void set_pwm_duty_cycle2(int duty) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

void control_pwm_cycles(int paces) {
    if (paces < 0) {
        gpio_set_level(GPIO_DIR_CONTROL, 0);
        paces = paces*-1;
    }
    else {
        gpio_set_level(GPIO_DIR_CONTROL, 1);
    }
    set_pwm_duty_cycle(PWM_DUTY_CYCLE * 1023 / 100);  // Establecer ciclo de trabajo
    vTaskDelay(pdMS_TO_TICKS(1000*paces/PWM_FREQUENCY));
    set_pwm_duty_cycle(0);                            // Poner ciclo de trabajo a 0
}

void control_pwm_cycles2(int paces) {
    if (paces < 0) {
        gpio_set_level(GPIO_DIR_CONTROL2, 0);
        paces = paces*-1;
    }
    else {
        gpio_set_level(GPIO_DIR_CONTROL2, 1);
    }
    // Subida del ciclo: Ciclo de trabajo al 50%
    set_pwm_duty_cycle2(PWM_DUTY_CYCLE * 1023 / 100);  // Establecer ciclo de trabajo
    vTaskDelay(pdMS_TO_TICKS(1000*paces/PWM_FREQUENCY2));

    // Bajada del ciclo: Ciclo de trabajo a 0
    set_pwm_duty_cycle2(0);  
}

void publisher_function(const char* message)
{
	// Update string message with new value and size before its published.
	char msg_buffer[50];
	msg.data.data = msg_buffer;
	msg.data.size = snprintf(msg.data.data, sizeof(msg_buffer), message);
	msg.data.capacity = sizeof(msg_buffer);
	rcl_publish(&esp32_publisher, &msg, NULL);
}

void subscriber1_callback(const void * msgin)
{
	const geometry_msgs__msg__Vector3 * msg = (const geometry_msgs__msg__Vector3 *)msgin;
    float x = msg->x;
    float y = msg->y;
    float z = msg->z;

	ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "Driver started");
	
	// Driver 1
    if (-62.0 <= x && x <= 62.0){
        m1_ang.num = (int)round(conv_rel_paces_angle1*(x - pos_m1));//conv_rel_paces_angle1*
        pos_m1 = x;
	    move_motor_can(1, m1_ang);
    }
	// Driver 2
    if (-100.0 <= y && y <= 100.0){
        m2_ang.num = (int)round(conv_rel_paces_angle2*(y - pos_m2));//conv_rel_paces_angle2*
        pos_m2 = y;
	    move_motor_can(2, m2_ang);
    }
	//Wait for bus to become free
	twai_status_info_t status_info;
	twai_get_status_info(&status_info);
	while (status_info.msgs_to_tx > 0) {
		vTaskDelay(pdMS_TO_TICKS(100));
		twai_get_status_info(&status_info);
	}
	ESP_ERROR_CHECK(twai_stop());
    ESP_LOGI(TAG, "Driver stopped");
    
    if (0.0 <= z && z <= 360.0){
        int paces_m3 = (int)round(conv_rel_paces_angle3*(z - pos_m3));
        pos_m3 = z;
        control_pwm_cycles2(paces_m3);
    }

	publisher_function("uros OK 1");
}

void subscriber2_callback(const void * msgin)
{
	const geometry_msgs__msg__Vector3 * msg = (const geometry_msgs__msg__Vector3 *)msgin;
    float x = msg->x;
    float y = msg->y;
    float z = msg->z;

    if (0.0 <= x && x <= 25.4){
        int paces_m4 = (int)round(conv_rel_paces_cm*(x - pos_m4));//conv_rel_paces_cm*
        pos_m4 = x; 
	    control_pwm_cycles(paces_m4); // minimo 50 pasos
    }
    if (y == 1.0){
        ESP_ERROR_CHECK(twai_start());
        ESP_LOGI(TAG, "Driver started");
        m1_ang.num = -10*conv_rel_paces_angle1;//conv_rel_paces_angle1*
	    move_motor_can(1, m1_ang);
        vTaskDelay(pdMS_TO_TICKS(200));
        m2_ang.num = -10*conv_rel_paces_angle2;//conv_rel_paces_angle2*
	    move_motor_can(2, m2_ang);
        //Wait for bus to become free
        twai_status_info_t status_info;
        twai_get_status_info(&status_info);
        while (status_info.msgs_to_tx > 0) {
		    vTaskDelay(pdMS_TO_TICKS(100));
		    twai_get_status_info(&status_info);
	    }
        ESP_ERROR_CHECK(twai_stop());
        ESP_LOGI(TAG, "Driver stopped");
    }
    if(z == 1.0){
        can_home();
    }
    
	publisher_function("uros OK 2");
}

void micro_ros_task(void * arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;
	// Create init_options.
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));
	#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
		rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
		// Static Agent IP and port can be used instead of autodisvery.
		RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
		//RCCHECK(rmw_uros_discover_agent(rmw_options));
	#endif
	// Setup support structure.
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
	// Create node.
	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "esp32_scara", "", &support));
	// Create subscriber.
	RCCHECK(rclc_subscription_init_default(
		&motor_angle_subscriber1,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
		"req_motor_ang1"));
	RCCHECK(rclc_subscription_init_default(
		&motor_angle_subscriber2,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
		"req_motor_ang2"));
	// Create publisher.
	RCCHECK(rclc_publisher_init_default(
		&esp32_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
		"esp32_message"));
	// Create executor.
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	unsigned int rcl_wait_timeout = 1000;   // in ms
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));
	// Add subscriber to executor.
	RCCHECK(rclc_executor_add_subscription(&executor, &motor_angle_subscriber1, &motor_angle1, &subscriber1_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &motor_angle_subscriber2, &motor_angle2, &subscriber2_callback, ON_NEW_DATA));
    // Spin forever.
	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(100000);
	}
	// Free resources.
	RCCHECK(rcl_subscription_fini(&motor_angle_subscriber1, &node));
	RCCHECK(rcl_subscription_fini(&motor_angle_subscriber2, &node));
	RCCHECK(rcl_publisher_fini(&esp32_publisher, &node));
	RCCHECK(rcl_node_fini(&node));
	//Uninstall TWAI driver
    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGI(TAG, "Driver uninstalled");
    vTaskDelete(NULL);
}

void app_main(void)
{
	for (int i = 3; i > 0; i--) {
        printf("Program starting in %d\n", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    init_gpio();
    init_pwm();
	#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
		ESP_ERROR_CHECK(uros_network_interface_initialize());
	#endif
	//Install TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(TAG, "Driver installed");
    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
	// Establecer el pin de salida digital en HIGH
    gpio_set_level(GPIO_DIR_CONTROL, 1); // para arriba el efector final
    while(1){
        int level = gpio_get_level(SENSOR); // Lee el nivel del pin (0 o 1)
        if (level == 1) {
            set_pwm_duty_cycle(PWM_DUTY_CYCLE * 1023 / 100);  // Establecer ciclo de trabajo
        }
        else {
            set_pwm_duty_cycle(0); 
            break;
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);
}



