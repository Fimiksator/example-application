/**
 ***********************************************************************
 * @file   main.cpp 
 * @author Fimiksator
 * @date   18/01/2025
 * @brief
 ***********************************************************************
*/

/*--------------------------------------------------------------------*/
/*---------------------------- INCLUDES ------------------------------*/
/*--------------------------------------------------------------------*/
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/logging/log.h>

/*--------------------------------------------------------------------*/
/*------------------- LOCAL FUNCTIONS PROTOTYPES ---------------------*/
/*--------------------------------------------------------------------*/


/*--------------------------------------------------------------------*/
/*--------------------------- DEFINITIONS ----------------------------*/
/*--------------------------------------------------------------------*/


/*--------------------------------------------------------------------*/
/*----------------------------- CLASSES ------------------------------*/
/*--------------------------------------------------------------------*/

/**
 * @class ReadClass - Reads the input pin and does some work when
 * 	it changes it's state
 */
class ReadClass {
private:
	const struct gpio_dt_spec *inputPin;
    struct gpio_callback gpio_cb;             // GPIO callback structure
    struct k_work work;                       // Work structure for deferred processing
    bool last_state;                          // Store the last GPIO state

    void onWork();
    void triggerWork();
    static void gpio_callback_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
    static void work_handler(struct k_work *work_item);

public:
    int initPin( const struct gpio_dt_spec*);
};

/**
 * @brief GPIO callback handler triggered by pin state changes.
 *
 * This static function is called whenever the GPIO interrupt is triggered.
 * It retrieves the class instance and schedules work to process the GPIO state.
 *
 * @param dev Pointer to the GPIO device structure.
 * @param cb Pointer to the GPIO callback structure.
 * @param pins Bitmask indicating which pins triggered the interrupt.
 */
void ReadClass::gpio_callback_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ReadClass *instance = CONTAINER_OF(cb, ReadClass, gpio_cb); // Retrieve the class instance
    instance->triggerWork(); // Trigger the work item
}

/**
 * @brief Static work handler for processing GPIO events.
 *
 * This static function processes the work item. It retrieves the class instance
 * and invokes the member function for processing work.
 *
 * @param work_item Pointer to the work item structure.
 */
void ReadClass::work_handler(struct k_work *work_item)
{
    ReadClass *instance = CONTAINER_OF(work_item, ReadClass, work); // Retrieve the class instance
    instance->onWork(); // Call the member function to process the work
}

/**
 * @brief Schedule work for GPIO state processing.
 *
 * This function submits the work item from the interrupt context to ensure
 * that processing is deferred to a non-interrupt context.
 */
void ReadClass::triggerWork()
{
    k_work_submit(&work);
}

/**
 * @brief Process the GPIO state change.
 *
 * This function reads the current GPIO pin state, checks if it has changed,
 * and publishes the new state using zbus if there is a change.
 */
void ReadClass::onWork()
{
	bool current_state = gpio_pin_get_dt(inputPin) > 0;  // Read the current GPIO state

    if (current_state != last_state)
	{  
		// Only update if the state changed
        last_state = current_state;

		bool state = gpio_pin_get_dt(inputPin) > 0;

		int ret = zbus_chan_pub(&gpio_state_chan, &current_state, K_NO_WAIT);
		if (ret == 0)
		{
			printk("Published GPIO state change: pin=%d, state=%s\n",
				inputPin->pin, state ? "HIGH" : "LOW");
		}
		else
		{
			printk("Failed to publish GPIO state: error=%d\n", ret);
		}
	}
}

/**
 * @brief Initialize the input GPIO pin with interrupts and work processing.
 *
 * Configures the input GPIO pin as an interrupt source, sets up a callback,
 * initializes the last state, and prepares the work item for processing events.
 *
 * @param pin Pointer to the GPIO descriptor structure.
 * @return 1 if initialization is successful, 0 otherwise.
 */
int ReadClass::initPin( const struct gpio_dt_spec* pin)
{
	inputPin = pin;

	int ret;

	if (!gpio_is_ready_dt( inputPin)) 
    {
        return 0;
	}

	ret = gpio_pin_configure_dt( inputPin, GPIO_INPUT);
	if ( ret != 0)
	{
		return 0;
	}

    /* Configure the GPIO interrupt */
    ret = gpio_pin_interrupt_configure_dt(inputPin, GPIO_INT_EDGE_BOTH); // Trigger on both edges
    if (ret != 0)
	{
        printk("Error %d: Failed to configure interrupt on pin %d\n", ret, inputPin->pin);
        return 0;
    }

    /* Initialize the callback structure */
    gpio_init_callback(&gpio_cb, gpio_callback_handler, BIT(inputPin->pin));
    gpio_add_callback(inputPin->port, &gpio_cb);

    /* Initialize last_state */
    last_state = gpio_pin_get_dt(inputPin) > 0;

	k_work_init(&work, work_handler);

    printk("GPIO pin %d initialized with interrupt, debounce, and work\n", inputPin->pin);

	return 1;
}
/*--------------------------------------------------------------------*/
/*-------------------------- GLOBAL VARIABLES ------------------------*/
/*--------------------------------------------------------------------*/

/*--------------------------------------------------------------------*/
/*---------------------------- FUNCTIONS -----------------------------*/
/*--------------------------------------------------------------------*/

int main(void)
{


	return 1;
}
