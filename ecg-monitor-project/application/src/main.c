#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h> 
#include <zephyr/logging/log.h>  // needs CONFIG_LOG=y in your prj.conf
#include <zephyr/drivers/adc.h> // CONFIG_ADC=y
#include <zephyr/drivers/pwm.h> // CONFIG_PWM=y
#include <zephyr/smf.h> // CONFIG_SMF=y
#include <zephyr/drivers/sensor.h> // CONFIG_SENSOR=y
#include "ecg_cycles.h"
#include "ble-lib.h"


LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);


//define macros 
#define HEARTBEAT_DUTY_1 500
#define HEARTBEAT_DUTY_2 500
#define ERROR_LED_DUTY_1 500
#define ERROR_LED_DUTY_2 500
#define ECG_DUTY_CYCLE .25
#define SAMPLE_INTERVAL_US 5000 // interval between samples in microseconds
#define BUFFER_ARRAY_LEN 800  // number of samples to collect in one ADC read operation (can change this make sure it fits on the board)
#define ECG_TIMER_MSEC 4000 // ECG measurement timer interval in ms
#define THRESHOLD 350

//define events
K_EVENT_DEFINE(action_events);
#define IDLE_RETURN_BUTTON_PRESS BIT(1)
#define TEMP_BUTTON_PRESS BIT(2)
#define ECG_BUTTON_PRESS BIT(3)
#define RESET_BUTTON_PRESS BIT(4)
#define BATTERY_MEASUREMENT_EVENT BIT(5)
#define BUFFER_FULL_EVENT BIT(6)

//define error events
K_EVENT_DEFINE(errors);
#define INIT_ERROR BIT(1)
#define TEMP_ERROR BIT(2)
#define ECG_ERROR BIT(3)
#define BATTERY_ERROR BIT(4)
#define BLE_ERROR BIT(5)
#define ADC_ERROR BIT(6)



#define ALL_BUTTON_PRESS (IDLE_RETURN_BUTTON_PRESS | RESET_BUTTON_PRESS | TEMP_BUTTON_PRESS| ECG_BUTTON_PRESS)

// ADC configuration parameters
#define ADC_DT_SPEC_GET_BY_ALIAS(adc_alias)                    \
{                                                            \
    .dev = DEVICE_DT_GET(DT_PARENT(DT_ALIAS(adc_alias))),      \
    .channel_id = DT_REG_ADDR(DT_ALIAS(adc_alias)),            \
    ADC_CHANNEL_CFG_FROM_DT_NODE(DT_ALIAS(adc_alias))          \
}                                                            \

// define globals and DT-based hardware structs
static const struct adc_dt_spec adc_vadc = ADC_DT_SPEC_GET_BY_ALIAS(vadc);
static const struct adc_dt_spec adc_diff = ADC_DT_SPEC_GET_BY_ALIAS(diffch);

//defining PWM DT-based hardware structs
static const struct pwm_dt_spec pwm1 = PWM_DT_SPEC_GET(DT_ALIAS(pwm1));
static const struct pwm_dt_spec pwm2 = PWM_DT_SPEC_GET(DT_ALIAS(pwm2));

//defining struct for temperature sensor
const struct device *const temp_sensor = DEVICE_DT_GET_ONE(jedec_jc_42_4_temp);
//configure button GPIO DT-based hardware structs
const struct gpio_dt_spec idle_return_button = GPIO_DT_SPEC_GET(DT_ALIAS(idlereturnbutton), gpios);
const struct gpio_dt_spec ecg_button = GPIO_DT_SPEC_GET(DT_ALIAS(ecgbutton), gpios);
const struct gpio_dt_spec temp_button = GPIO_DT_SPEC_GET(DT_ALIAS(tempbutton), gpios);
const struct gpio_dt_spec reset_button = GPIO_DT_SPEC_GET(DT_ALIAS(resetbutton), gpios);

//configure LED GPIO DT-based hardware structs
const struct gpio_dt_spec heartbeat_led = GPIO_DT_SPEC_GET(DT_ALIAS(heartbeat), gpios);
const struct gpio_dt_spec error_led = GPIO_DT_SPEC_GET(DT_ALIAS(error), gpios);
//const struct gpio_dt_spec battery_led = GPIO_DT_SPEC_GET(DT_ALIAS(battery), gpios);
const struct gpio_dt_spec ecg_led = GPIO_DT_SPEC_GET(DT_ALIAS(ecg), gpios);

// define callback functions
void idle_return_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void temp_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void ecg_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void reset_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

// initialize GPIO Callback Structs
static struct gpio_callback idle_return_button_cb; 
static struct gpio_callback temp_button_cb;
static struct gpio_callback ecg_button_cb;
static struct gpio_callback reset_button_cb; 

// Note that the buffer must be int16_t (16-bit integer) to store the ADC data
int16_t buf;
int16_t buf_diff[BUFFER_ARRAY_LEN];
int16_t buf_update[BUFFER_ARRAY_LEN];

struct adc_sequence sequence = {
    .buffer = &buf,
    .buffer_size = sizeof(buf), // bytes
};

//Forward declarations
enum adc_action diff_adc_callback(const struct device *dev,const struct adc_sequence *sequence_diff, uint16_t sampling_index);
static int battery_measurement();
int read_temperature_sensor(const struct device *temp_sensor, int32_t *temperature_degC);

struct adc_sequence_options options = {
        .extra_samplings = BUFFER_ARRAY_LEN - 1,  // -1 b/c first sample is already in the buffer
        .interval_us = SAMPLE_INTERVAL_US,  // 1000 us between samples
        .callback = diff_adc_callback,  // called after each sample is collected
};


struct adc_sequence sequence_diff = {
    .options = &options,
    .buffer = &buf_diff,
    .buffer_size = BUFFER_ARRAY_LEN * sizeof(buf_diff[0])
};

//Global variables
int32_t temperature_degC;

int num_of_cycles = 0;
int on_time = 500; //making system blink at 1Hz while first buffer is filling
int off_time = 500;
int heart_rate = 0;
static uint32_t last_error = 0;


//defining timers and their handlers
void timer_interval_battery_measurement(struct k_timer *battery_measurement_timer);
K_TIMER_DEFINE(battery_measurement_timer, timer_interval_battery_measurement, NULL);

void timer_interval_ecg_measurement(struct k_timer *ecg_measurement_timer);
K_TIMER_DEFINE(ecg_measurement_timer, timer_interval_ecg_measurement, NULL);
/* User defined object */
struct s_object {
        /* This must be first */
        struct smf_ctx ctx;

} s_obj;


/* Forward declaration of state table */ 
static const struct smf_state demo_states[]; 

enum demo_state {INIT, IDLE, HEARTRATE, ERROR};

//defining threads

//heartbeat thread
extern void heartbeat_thread(void *, void *, void *); 
K_THREAD_DEFINE(heartbeat_thread_id, 1024, heartbeat_thread, NULL, NULL, NULL, 5, 0, 0);

extern void heartbeat_thread(void *, void *, void *) {

    while (1) {
        k_msleep(HEARTBEAT_DUTY_1);  // scheduler can run other tasks now
        gpio_pin_toggle_dt(&heartbeat_led);
        k_msleep(HEARTBEAT_DUTY_2); // scheduler can run other tasks now
        gpio_pin_toggle_dt(&heartbeat_led);
        LOG_INF("Heartbeat LED toggled");
    } 

}
//error LED thread
extern void error_led_thread(void *, void *, void *); 

K_THREAD_DEFINE(error_thread_id, 1024, error_led_thread, NULL, NULL, NULL, 4, 0, 0);

extern void error_led_thread(void *, void *, void *) {

    while (1) {
        //pattern for all LEDs
        k_msleep(ERROR_LED_DUTY_1);  // scheduler can run other tasks now
        gpio_pin_set_dt(&heartbeat_led,1);
        gpio_pin_set_dt(&error_led,1);
        gpio_pin_set_dt(&ecg_led,1);
        pwm_set_pulse_dt(&pwm1, pwm1.period*1); // set to max duty cycle
        k_msleep(ERROR_LED_DUTY_2); // scheduler can run other tasks now
        gpio_pin_set_dt(&heartbeat_led,0);
        gpio_pin_set_dt(&error_led,0);
        gpio_pin_set_dt(&ecg_led,0);
        pwm_set_pulse_dt(&pwm1, 0);
        LOG_INF("Error LEDS toggled");
    } 

}


//error watch thread
extern void error_watch_thread(void *, void *, void *); 
K_THREAD_DEFINE(error_watch_thread_id, 1024, error_watch_thread, NULL, NULL, NULL, 6, 0, 0);

extern void error_watch_thread(void *, void *, void *) {
    while (1) {
        int32_t events = k_event_wait(&errors, (INIT_ERROR | TEMP_ERROR | ECG_ERROR | BATTERY_ERROR | BLE_ERROR), true, K_FOREVER);

        // notification 
        if (events) {
            last_error = events;
            LOG_INF("last error is %d", last_error);
            bluetooth_notify_errors(last_error);
            smf_set_state(&s_obj.ctx, &demo_states[ERROR]);
        }
    }
}
//heartrate calculation thread
void heartrate_calculation_thread(void *, void *, void *);
K_THREAD_DEFINE(heartrate_calculation_thread_id, 2048, heartrate_calculation_thread, NULL, NULL, NULL, 5, 0, 0);
void heartrate_calculation_thread(void *, void *, void *) {
    while (1) {
        
        gpio_pin_set_dt(&ecg_led, 1);
        k_msleep(on_time);
        gpio_pin_set_dt(&ecg_led, 0);
        k_msleep(off_time);
    }  
}


/*init state*/
static void init_run(void *o) 
{   
    // check if interface is ready
    if (!device_is_ready(idle_return_button.port)) {
        LOG_ERR("idle return button interface not ready."); 
        k_event_post(&errors, INIT_ERROR); 
        return;  
    }

    if (!device_is_ready(reset_button.port)) {
        LOG_ERR("reset button interface not ready.");
        k_event_post(&errors, INIT_ERROR);
        return; 
    }

    if (!device_is_ready(temp_button.port)) {
        LOG_ERR("temp button interface not ready.");
        k_event_post(&errors, INIT_ERROR); 
        return;
    }

    if (!device_is_ready(ecg_button.port)) {
        LOG_ERR("ecg button interface not ready.");
        k_event_post(&errors, INIT_ERROR); 
        return;
    }
    
    if (!device_is_ready(heartbeat_led.port)) {
        LOG_ERR("heartbeat LED interface not ready.");
        k_event_post(&errors, INIT_ERROR);  
        return;  
    }

    if (!device_is_ready(error_led.port)) {
        LOG_ERR("error LED interface not ready."); 
        k_event_post(&errors, INIT_ERROR);
        return;  
    }
    if (!device_is_ready(ecg_led.port)) {
        LOG_ERR("ecg LED interface not ready.");
        k_event_post(&errors, INIT_ERROR);  
        return;  
    }
    
    // configure GPIO pin
    int err = gpio_pin_configure_dt(&idle_return_button, GPIO_INPUT);
    if (err < 0) {
        LOG_ERR("Cannot configure idle return button pin.");
        k_event_post(&errors, INIT_ERROR);
        return;
    }
    err = gpio_pin_configure_dt(&reset_button, GPIO_INPUT);
    if (err < 0) {
        LOG_ERR("Cannot configure reset button pin.");
        k_event_post(&errors, INIT_ERROR);
        return;
    }
    
    err = gpio_pin_configure_dt(&temp_button, GPIO_INPUT);
    if (err < 0) {
        LOG_ERR("Cannot configure temp_button pin.");
        k_event_post(&errors, INIT_ERROR);
        return;
    }

    err = gpio_pin_configure_dt(&ecg_button, GPIO_INPUT);
    if (err < 0) {
        LOG_ERR("Cannot configure ecg_button pin.");
        k_event_post(&errors, INIT_ERROR);
        return;
    }

    err = gpio_pin_configure_dt(&heartbeat_led, GPIO_OUTPUT_ACTIVE);
    if (err < 0) {
        LOG_ERR("Cannot configure heartbeat LED.");
        k_event_post(&errors, INIT_ERROR);
        return;
    }

    err = gpio_pin_configure_dt(&error_led, GPIO_OUTPUT_ACTIVE);
    if (err < 0) {
        LOG_ERR("Cannot configure error LED.");
        k_event_post(&errors, INIT_ERROR);
        return;
    }

    err = gpio_pin_configure_dt(&ecg_led, GPIO_OUTPUT_ACTIVE);
    if (err < 0) {
        LOG_ERR("Cannot configure iv pump LED.");
        k_event_post(&errors, INIT_ERROR);
        return;
    }

    // trigger on transition from INACTIVE -> ACTIVE; ACTIVE could be HIGH or LOW
    err = gpio_pin_interrupt_configure_dt(&idle_return_button, GPIO_INT_EDGE_TO_ACTIVE); 
    if (err < 0) {
        LOG_ERR("Cannot attach callback to idle return button.");
        k_event_post(&errors, INIT_ERROR);
    }
    err = gpio_pin_interrupt_configure_dt(&reset_button, GPIO_INT_EDGE_TO_ACTIVE); 
    if (err < 0) {
        LOG_ERR("Cannot attach callback to reset button.");
        k_event_post(&errors, INIT_ERROR);
    }
    err = gpio_pin_interrupt_configure_dt(&temp_button, GPIO_INT_EDGE_TO_ACTIVE);
    if (err < 0) {
        LOG_ERR("Cannot attach callback to temp_button.");
        k_event_post(&errors, INIT_ERROR);
    }
    err = gpio_pin_interrupt_configure_dt(&ecg_button, GPIO_INT_EDGE_TO_ACTIVE);
    if (err < 0) {
        LOG_ERR("Cannot attach callback to ecg_button.");
        k_event_post(&errors, INIT_ERROR);
    }
    
    // populate CB struct with information about the CB function and pin
    gpio_init_callback(&idle_return_button_cb, idle_return_button_callback, BIT(idle_return_button.pin)); // associate callback with GPIO pin
    gpio_init_callback(&reset_button_cb, reset_button_callback, BIT(reset_button.pin)); // associate callback with GPIO pin
    gpio_init_callback(&temp_button_cb, temp_button_callback, BIT(temp_button.pin));
    gpio_init_callback(&ecg_button_cb, ecg_button_callback, BIT(ecg_button.pin));

    gpio_add_callback_dt(&idle_return_button, &idle_return_button_cb);
    gpio_add_callback_dt(&reset_button, &reset_button_cb);
    gpio_add_callback_dt(&temp_button, &temp_button_cb);
    gpio_add_callback_dt(&ecg_button, &ecg_button_cb);

    // check if ADC device is ready
    if (!device_is_ready(adc_vadc.dev)) {
        LOG_ERR("ADC controller device(s) not ready");
        k_event_post(&errors, INIT_ERROR);
        return;
    }

    if (!device_is_ready(adc_diff.dev)) {
        LOG_ERR("ADC diff controller device(s) not ready");
        k_event_post(&errors, INIT_ERROR);
        return;
    }

    // check that the PWM controller is ready
    if (!device_is_ready(pwm1.dev))  {
        LOG_ERR("PWM device %s is not ready.", pwm1.dev->name);
        k_event_post(&errors, INIT_ERROR);
        return;
    }

    if (!device_is_ready(pwm2.dev))  {
        LOG_ERR("PWM device %s is not ready.", pwm2.dev->name);
        k_event_post(&errors, INIT_ERROR);
        return;
    }

    /* Configure the ADC channel */
    err = adc_channel_setup_dt(&adc_vadc);
    if (err < 0) {
        LOG_ERR("Could not setup ADC channel (%d)", err);
        k_event_post(&errors, INIT_ERROR);
        return;
    }

    err = adc_channel_setup_dt(&adc_diff);
    if (err < 0) {
        LOG_ERR("Could not setup ADC diff channel (%d)", err);
        k_event_post(&errors, INIT_ERROR);
        return;
    }
    /*
    //Checking to see if the sensor is ready
    if (!device_is_ready(temp_sensor)) {
        LOG_ERR("Temperature sensor %s is not ready", temp_sensor->name);
        k_event_post(&errors, INIT_ERROR);
        return;
    }
   else {
        LOG_INF("Temperature sensor %s is ready", temp_sensor->name);
    }
    */
    //suspending threads
    k_thread_suspend(error_thread_id);
    k_thread_suspend(heartrate_calculation_thread_id);
    
    //turn off LEDs
    gpio_pin_set_dt(&error_led, 0);
    gpio_pin_set_dt(&ecg_led, 0);
    pwm_set_pulse_dt(&pwm1, 0);  // Set pulse to 0 (0% duty cycle)
    smf_set_state(&s_obj.ctx, &demo_states[IDLE]); // transition to next state

}

/*IDLE STATE*/

static void idle_entry(void *o) 
{ 
    LOG_INF("Idle state entry...");

    //turn off LEDs
    gpio_pin_set_dt(&error_led, 0);
    gpio_pin_set_dt(&ecg_led, 0);
    pwm_set_pulse_dt(&pwm1, 0);  // Set pulse to 0 (0% duty cycle)
    
    //enable button ISR/CB for all buttons
    gpio_pin_interrupt_configure_dt(&idle_return_button, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_pin_interrupt_configure_dt(&temp_button, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_pin_interrupt_configure_dt(&reset_button, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_pin_interrupt_configure_dt(&ecg_button, GPIO_INT_EDGE_TO_ACTIVE);

    //Perform initial battery measurement
    int ret = battery_measurement();
    if (ret < 0) {
        k_event_post(&errors, BATTERY_ERROR); 
        return;
    }
    
    //start the battery event timer
    k_timer_start(&battery_measurement_timer, K_SECONDS(60), K_SECONDS(60)); // start timer to trigger every 60 seconds
    
}
static void idle_run(void *o) 
{  
    LOG_INF("Idle state running...");
    int32_t events = k_event_wait(&action_events, (IDLE_RETURN_BUTTON_PRESS | TEMP_BUTTON_PRESS | ECG_BUTTON_PRESS | RESET_BUTTON_PRESS| BATTERY_MEASUREMENT_EVENT), true, K_FOREVER);
   
    if (events & IDLE_RETURN_BUTTON_PRESS) {
        LOG_INF("idle return button pressed, going to back to idle entry");
        smf_set_state(SMF_CTX(&s_obj), &demo_states[IDLE]); //transition to IDLE state
        return;
    } 

    if (events & TEMP_BUTTON_PRESS) {
        LOG_INF("temp button pressed, calculating the temperature");
        int ret;
        ret = read_temperature_sensor(temp_sensor, &temperature_degC);
        if (ret != 0) {
            LOG_ERR("There was a problem reading the temperature sensor (%d)", ret);
            k_event_post(&errors, TEMP_ERROR); //transition to ERROR STATE
            return;
        }
        LOG_INF("Temperature: %f", (double)temperature_degC);
        
    }  

    if (events & ECG_BUTTON_PRESS) {
        LOG_INF("ecg button pressed, going to ecg state");
        smf_set_state(SMF_CTX(&s_obj), &demo_states[HEARTRATE]);  // transition to the ECG state
        return;
    }

    if (events & RESET_BUTTON_PRESS) {
        LOG_INF("Reset button pressed, going to idle state");
        //reset all global variables
        temperature_degC = 0;
        num_of_cycles = 0;
        on_time = 500;
        off_time = 500;
        heart_rate = 0;
        LOG_INF("All variables reset");
        smf_set_state(SMF_CTX(&s_obj), &demo_states[IDLE]);  // transition to the IDLE state
        return;
    }

    if (events & BATTERY_MEASUREMENT_EVENT) {
        LOG_INF("Battery measurement event triggered");
    
        int ret = battery_measurement();
        if (ret < 0) {
            k_event_post(&errors, BATTERY_ERROR); 
            return;
        }
    }
    
}

static void idle_exit(void *o)
{
    //stop battery measurement timer
    k_timer_stop(&battery_measurement_timer);
    LOG_INF("stopping battery measurement timer...");
    LOG_INF("Idle state exit...");
}

/*HEART RATE STATE*/
static void heartrate_entry(void *o)
{ //ensure all buttons are enabled
    LOG_INF("Heartrate state entry...");
    gpio_pin_interrupt_configure_dt(&idle_return_button, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_pin_interrupt_configure_dt(&temp_button, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_pin_interrupt_configure_dt(&reset_button, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_pin_interrupt_configure_dt(&ecg_button, GPIO_INT_EDGE_TO_ACTIVE);

    // turn off battery LED
    pwm_set_pulse_dt(&pwm1, 0);  // Set pulse to 0 (0% duty cycle)

    k_timer_start(&ecg_measurement_timer, K_MSEC(ECG_TIMER_MSEC), K_MSEC(ECG_TIMER_MSEC)); // start timer to trigger every ECG_TIMER_MSEC milliseconds
    //resume threads
    k_thread_resume(heartrate_calculation_thread_id);
}
static void heartrate_run(void *o)
{   
   
    LOG_INF("Heartrate state running...");
    
    LOG_INF("Measuring %s (channel %d)... ", adc_diff.dev->name, adc_diff.channel_id);

    (void)adc_sequence_init_dt(&adc_diff, &sequence_diff);

    int ret;
    ret = adc_read_async(adc_diff.dev, &sequence_diff, NULL);

    if (ret < 0) {
        LOG_ERR("Could not read (%d)", ret);
        k_event_post(&errors, ADC_ERROR); 
        return;
    }

    
    int32_t events = k_event_wait(&action_events, RESET_BUTTON_PRESS | TEMP_BUTTON_PRESS | IDLE_RETURN_BUTTON_PRESS | ECG_BUTTON_PRESS | BUFFER_FULL_EVENT, true, K_FOREVER);

    if (events & BUFFER_FULL_EVENT) {
        LOG_INF("buffer full event posted");
          
    } 

    if (events & TEMP_BUTTON_PRESS) {
        LOG_INF("temp button pressed, calculating the temperature");
        int ret;
        ret = read_temperature_sensor(temp_sensor, &temperature_degC);
        if (ret != 0) {
            LOG_ERR("There was a problem reading the temperature sensor (%d)", ret);
            k_event_post(&errors, TEMP_ERROR);
            return;
        }
        LOG_INF("Temperature: %f", (double)temperature_degC);   
    } 

    if (events & RESET_BUTTON_PRESS) {
        LOG_INF("Reset button pressed, going to init mode");
        //reset all global variables
        temperature_degC = 0;
        num_of_cycles = 0;
        on_time = 500;
        off_time = 500;
        heart_rate = 0;
        LOG_INF("All variables reset");
        smf_set_state(SMF_CTX(&s_obj), &demo_states[INIT]);  // transition to the INIT state
        return;
    }

    if (events & IDLE_RETURN_BUTTON_PRESS) {
        LOG_INF("idle return button pressed, going to idle state");
        //clear all variables and set them to 0
        smf_set_state(SMF_CTX(&s_obj), &demo_states[IDLE]);  // transition to the IDLE state
        return;
    }

    if (events & ECG_BUTTON_PRESS) {
        LOG_INF("ecg button pressed, returning to idle state");
        smf_set_state(SMF_CTX(&s_obj), &demo_states[IDLE]);  // transition to the IDLE state
        return;
    }
}

static void heartrate_exit(void *o)
{
    //suspend threads here
    k_thread_suspend(heartrate_calculation_thread_id);
    k_timer_stop(&ecg_measurement_timer);
    LOG_INF("Heartrate state exit... entering IDLE");
}

/*ERROR STATE*/
static void error_entry(void *o)
{
    LOG_INF("Error state entry...");
    //suspend heartbeat thread
    k_thread_suspend(heartbeat_thread_id);

    //turn off all LEDs
    gpio_pin_set_dt(&heartbeat_led, 0);
    gpio_pin_set_dt(&ecg_led, 0);
    gpio_pin_set_dt(&error_led, 0);
    pwm_set_pulse_dt(&pwm1, 0);  // Set pulse to 0 (0% duty cycle)

    //disable button ISR/CB for all buttons except reset button
    gpio_pin_interrupt_configure_dt(&idle_return_button, GPIO_INT_DISABLE);
    gpio_pin_interrupt_configure_dt(&temp_button, GPIO_INT_DISABLE);
    gpio_pin_interrupt_configure_dt(&ecg_button, GPIO_INT_DISABLE);
}
static void error_run(void *o)
{   
    LOG_INF("Error state running...");
    //resume the error LED thread
    k_thread_resume(error_thread_id);
    
    int32_t button_event_data = k_event_wait(&action_events, RESET_BUTTON_PRESS, true, K_FOREVER);
    if (button_event_data & RESET_BUTTON_PRESS) {
        LOG_INF("Reset button pressed, going to init mode");
        //setting global variables to 0
        temperature_degC = 0;
        num_of_cycles = 0;
        on_time = 500;
        off_time = 500;
        heart_rate = 0;
        LOG_INF("All variables reset");
        smf_set_state(SMF_CTX(&s_obj), &demo_states[IDLE]);  // transition to the INIT state
    } 
}

static void error_exit(void *o)
{
    last_error = 0;
    //suspend error LED thread
    k_thread_suspend(error_thread_id);
    //resume heartbeat thread
    k_thread_resume(heartbeat_thread_id);
    LOG_INF("Error state exit... entering IDLE");
}

/* State table definition */
static const struct smf_state demo_states[] = { 

    [INIT] = SMF_CREATE_STATE(NULL, init_run, NULL, NULL, NULL), 

    [IDLE] = SMF_CREATE_STATE(idle_entry, idle_run, idle_exit, NULL, NULL), 

    [HEARTRATE] = SMF_CREATE_STATE(heartrate_entry, heartrate_run, heartrate_exit, NULL, NULL),

    [ERROR] = SMF_CREATE_STATE(error_entry, error_run, error_exit, NULL, NULL), 

}; 

int main(void)
{
      /* Set initial state */
    smf_set_initial(SMF_CTX(&s_obj), &demo_states[INIT]);
    int ret;

    ret = bluetooth_init(&bluetooth_callbacks, &remote_service_callbacks);
    if (ret != 0) {
            LOG_ERR("There was a problem setting bluetooth");
            smf_set_state(SMF_CTX(&s_obj), &demo_states[ERROR]);
        }
   

    /* Run the state machine */
    while(1) {
    
        /* State machine terminates if a non-zero value is returned */
        ret = smf_run_state(SMF_CTX(&s_obj));
        if (ret) {
            /* handle return code and terminate state machine */
            smf_set_terminate(SMF_CTX(&s_obj), ret);
            break;
        }
        k_msleep(10);
    }
    
}

// define callback functions
void idle_return_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_event_post(&action_events,IDLE_RETURN_BUTTON_PRESS);
                             
}

void reset_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_event_post(&action_events,RESET_BUTTON_PRESS);  
}

void temp_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_event_post(&action_events, TEMP_BUTTON_PRESS);  
}

void ecg_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_event_post(&action_events, ECG_BUTTON_PRESS);  
}


enum adc_action diff_adc_callback(const struct device *dev,const struct adc_sequence *sequence_diff, uint16_t sampling_index)
{   
    int ret;
    int32_t val_mv = buf_diff[sampling_index];
    ret = adc_raw_to_millivolts_dt(&adc_diff, &val_mv);
    if (ret < 0) {
        LOG_ERR("Buffer cannot be converted to mV");
        k_event_post(&errors, ADC_ERROR);
        return ret;
    }
    //LOG_DBG("ADC Buffer (mV): %d", val_mv);
    int32_t val_mv_prev = 0;
    
    if (sampling_index < (BUFFER_ARRAY_LEN -1)) {

        if (sampling_index >1){
            val_mv_prev = buf_diff[sampling_index-1];
            ret = adc_raw_to_millivolts_dt(&adc_diff, &val_mv_prev);
            if (ret < 0) {
                LOG_ERR("Buffer cannot be converted to mV");
                k_event_post(&errors, ADC_ERROR);
                return ret;
            }
        }

        if ((val_mv > THRESHOLD) && (val_mv_prev <= THRESHOLD)) {
            num_of_cycles = num_of_cycles + 1;
        }
    }

    else if (sampling_index == (BUFFER_ARRAY_LEN - 1)) {
        k_event_post(&action_events, BUFFER_FULL_EVENT);
        LOG_INF("Buffer full at index %d, finishing batch.", sampling_index);
        
        return ADC_ACTION_FINISH;
    }
    
    return ADC_ACTION_CONTINUE;
};

//Battery measurement helper function

static int battery_measurement(void)
{
    LOG_INF("Performing battery measurement...");
    LOG_INF("Measuring %s (channel %d)...", adc_vadc.dev->name, adc_vadc.channel_id);

    (void)adc_sequence_init_dt(&adc_vadc, &sequence);

    int ret = adc_read(adc_vadc.dev, &sequence);
    if (ret < 0) {
        LOG_ERR("Could not read ADC (%d)", ret);
        k_event_post(&errors, BATTERY_ERROR);
        return ret;
    } else {
        LOG_DBG("Raw ADC Buffer: %d", buf);
    }
    int32_t val_mv = buf;
    ret = adc_raw_to_millivolts_dt(&adc_vadc, &val_mv);
    if (ret < 0) {
        LOG_ERR("Buffer cannot be converted to mV");
        k_event_post(&errors, BATTERY_ERROR);
        return ret;
    } else {
        LOG_INF("ADC Value (mV): %d", val_mv);
    }

    float duty_cycle = (val_mv / 1000.0f) / 3.0f;
    if (duty_cycle > 1.0f) duty_cycle = 1.0f;
    if (duty_cycle < 0.0f) duty_cycle = 0.0f;
    
    uint32_t pulse = (uint32_t)(duty_cycle * pwm1.period);
    LOG_INF("Setting PWM duty cycle to %d%% (pulse = %u ns)", (int)(duty_cycle * 100), pulse);
    
    ret = pwm_set_pulse_dt(&pwm1, pulse);
    if (ret < 0) {
        LOG_ERR("Failed to set PWM (%d)", ret);
        k_event_post(&errors, BATTERY_ERROR);
        return ret;
    }
    bluetooth_set_battery_level(val_mv);
    return 0;
}

//temperature measurement helper function

int read_temperature_sensor(const struct device *temp_sensor, int32_t *temperature_degC) {
    
    struct sensor_value sensor_vals = {.val1 = 0, .val2 = 0};
    int err = sensor_sample_fetch(temp_sensor);
    bluetooth_notify_temperature();
    if (err != 0) {
        LOG_ERR("Temperature sensor fetch(): %d", err);
        return err;
    }
    else {
        // sensor channels: https://docs.zephyrproject.org/latest/doxygen/html/group__sensor__interface.html#gaaa1b502bc029b10d7b23b0a25ef4e934

        err = sensor_channel_get(temp_sensor, SENSOR_CHAN_AMBIENT_TEMP, &sensor_vals);
        if (err != 0) {
            LOG_ERR("Temperature sensor get(): %d", err);
            return err;
        }
    }
        
        // data returned in kPa
        *temperature_degC = sensor_value_to_float(&sensor_vals);
        LOG_INF("Temperature (deg C): %f", (double)*temperature_degC);
        return 0;
}

//define timer handler functions
void timer_interval_battery_measurement(struct k_timer *battery_measurement_timer)
{
    LOG_INF("measuring battery voltage...");
    //post an event to measure battery voltage
    k_event_post(&action_events, BATTERY_MEASUREMENT_EVENT);
}

void timer_interval_ecg_measurement(struct k_timer *ecg_measurement_timer)
{
    //LOG_HEXDUMP_INF(buf_diff, sizeof(buf_diff[0])*BUFFER_ARRAY_LEN, "Raw ADC Diff Buffer:");
    LOG_INF("number of cycles %d,", num_of_cycles);
    LOG_INF("measuring battery voltage...");
    //post an event to measure battery voltage
    heart_rate = num_of_cycles * (60000 / ECG_TIMER_MSEC); //beats per minute
    if (num_of_cycles >0) {
        int time_per_beat = ECG_TIMER_MSEC / num_of_cycles;
        // On/off times as portion of time per beat
        on_time = ECG_DUTY_CYCLE * time_per_beat;  // 0.25 * time_per_beat
        off_time = (1 - ECG_DUTY_CYCLE) * time_per_beat;  // 0.75 * time_per_beat

        //check for out of range heart rate
        if (heart_rate < 40 || heart_rate > 200) {
        LOG_ERR("Calculated heart rate out of range: %d", heart_rate);
        k_event_post(&errors, ECG_ERROR);
        return;
        }
        bt_hrs_notify(heart_rate);
        num_of_cycles = 0;
  
    }
    
    LOG_INF("ontime is %d", on_time);
    LOG_INF("offtime is %d", off_time);
    LOG_INF("logging heartrate at %d", heart_rate);
}



