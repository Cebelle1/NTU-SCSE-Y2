#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>

static unsigned int ledGreen = 4; // GPIO4 connected to Green LED
static unsigned int pushButton = 11; // GPIO11 connected to push Button
static unsigned int irqNumber; // share IRQ number within file
static bool ledOn = 0; // used to toggle state of LED

// The GPIO IRQ Handler function prototype
static irq_handler_t rpi_gpio_isr(unsigned int irq, void *dev_id, struct pt_regs *regs);

// The GPIO IRQ Handler function
static irq_handler_t rpi_gpio_isr(unsigned int irq, void *dev_id, struct pt_regs *regs) {
    ledOn = !ledOn; // Toggle the LED state
    gpio_set_value(ledGreen, ledOn); // Set the LED accordingly
    printk(KERN_ALERT "GPIO Interrupt!\n");
    return (irq_handler_t) IRQ_HANDLED; // Announce that the IRQ has been handled
}

// The LKM initialization function
static int __init rpi_gpio_init(void) {
    int result = 0;
    printk(KERN_ALERT "Initializing the GPIO LKM\n");
    ledOn = true; // Default the LED to ON

    // Request and configure GPIOs
    gpio_request(ledGreen, "sysfs");
    gpio_direction_output(ledGreen, ledOn); // Set in output mode and turn on LED initially
    gpio_request(pushButton, "sysfs");
    gpio_direction_input(pushButton); // Set up as input
    gpio_set_debounce(pushButton, 200); // Debounce delay of 200ms

    // Map GPIO to IRQ number
    irqNumber = gpio_to_irq(pushButton);
    printk(KERN_ALERT "Button mapped to IRQ: %d\n", irqNumber);

    // Request for an interrupt line
    result = request_irq(irqNumber, // Interrupt number requested
                         (irq_handler_t) rpi_gpio_isr, // ISR handler function
                         IRQF_TRIGGER_RISING, // Trigger on rising edge
                         "rpi_gpio_handler", // Used in /proc/interrupts
                         NULL); // *dev_id for shared interrupt lines, NULL in this case
    return result;
}

// The LKM exit function
static void __exit rpi_gpio_exit(void) {
    gpio_set_value(ledGreen, 0); // Turn the LED off
    gpio_free(ledGreen); // Free the LED GPIO
    gpio_free(pushButton); // Free the Button GPIO
    free_irq(irqNumber, NULL); // Free the IRQ number, no *dev_id
    printk(KERN_ALERT "Goodbye from the GPIO LKM!\n");
}

module_init(rpi_gpio_init); // Macro to execute module's initialize routine
module_exit(rpi_gpio_exit); // Macro to execute module's exit routine

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Cebelle");
MODULE_DESCRIPTION("A simple Linux GPIO Interrupt LKM for Raspberry Pi");
MODULE_VERSION("0.1");
