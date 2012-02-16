#include <linux/kernel.h>
#include <linux/module.h>

MODULE_LICENSE("GPL");

static int __init qq2440_hello_module_init(void)
{
    printk("Hello, QQ2440 module is installed !\n");
    return 0;
}

static void __exit qq2440_hello_module_cleanup(void)
{
    printk("Good-bye, QQ2440 module was removed!\n");
}

module_init(qq2440_hello_module_init);
module_exit(qq2440_hello_module_cleanup);
