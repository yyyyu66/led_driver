#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/uaccess.h> 
#include <asm/io.h>
#include <linux/device.h> 

//#include <linux/gpio.h>
//#include <mach/gpio.h>
//#include <plat/gpio-cfg.h>

#define  DEVICE_NAME  "led_linux2.6"

// 1.����LED��Ӧ��IO��ΪGPC0_3         GPC0_4
#define  BASE_ADDR  (0XE0200060)
static volatile unsigned int * base_addr = NULL; //
#define  GPC0CON    (*(volatile unsigned int *)(base_addr + 0))  //0
#define  GPC0DAT    (*(volatile unsigned int *)(base_addr + 1))  //4 
#define  GPC0PUD    (*(volatile unsigned int *)(base_addr + 2))  //4 
#define  GPC0DRV    (*(volatile unsigned int *)(base_addr + 3))  //4 
#define  GPC0CONPDN (*(volatile unsigned int *)(base_addr + 4))  //4 
//GPC0PUD GPC0DRV GPC0CONPDN������Ĵ������Բ�������

//�豸�Ĳ�������
// 1.����Ϊ���
static int led_open(struct inode *pinode, struct file *pfile)
{
	int ret;
	
	base_addr = (unsigned int *)ioremap(BASE_ADDR,0x14);  
	if(!base_addr)
	{
		printk("ioremap error.\n");
		ret = -ENOMEM;
		return ret
;
	}
	GPC0CON &= ~(0XFF << 12);  
	GPC0CON |=  (0X11 << 12);    	
	GPC0DAT &= ~(0X3 << 3);      
	GPC0DAT |=  (0X3 << 3);    

	printk(KERN_EMERG"line:%d, %s is call\n", __LINE__, __FUNCTION__);
	return 0;
}

static int led_close(struct inode *pinode, struct file *pfile)
{
	GPC0DAT &= ~(0X3 << 3); 

	iounmap(base_addr); //ӳ��ĵ�ַ�ͷ�
	printk(KERN_EMERG"line:%d, %s is call\n", __LINE__, __FUNCTION__);
	return 0;
}

static ssize_t led_read(struct file *pfile, char __user * buf , size_t count, loff_t * poff)
{
	printk(KERN_EMERG"line:%d, %s is call\n", __LINE__, __FUNCTION__);
	return 0;
}

static size_t  led_write(struct file *pfile, const char __user *buff, size_t count, loff_t *poff)
{
	//����豸
	printk(KERN_EMERG"line:%d, %s is call\n", __LINE__, __FUNCTION__);
	return 0;
}

static long led_ioctl(struct file *pfile, unsigned int cmd, unsigned long arg)
{
	printk(KERN_EMERG"line:%d, %s is call\n", __LINE__, __FUNCTION__);
	return 0;
}


//�ļ�����ָ��
static struct file_operations button_fops = {
	.open    		= led_open,
	.read    		= led_read,
	.write   		= led_write,
	.release 		= led_close,
	.unlocked_ioctl = led_ioctl,		
};

static unsigned int major = 0;
static struct cdev * led_cdev;
static dev_t dev_number;
static struct class * led_class;
static struct device * led_device;

static int __init led_init(void)
{
	int ret = 0;
	
	// 1.�������cdev�ռ�
	led_cdev = cdev_alloc();
	if(led_cdev == NULL)
	{
		printk(KERN_WARNING"%d:cdev_alloc error.\n", __LINE__);
		ret = -ENOMEM;
		goto ERROR1;
	}
	// 2.��̬�����豸��
	ret = alloc_chrdev_region(&dev_number, 0, 4, DEVICE_NAME);
	if(ret < 0){
		printk(KERN_WARNING"%d:alloc_chrdev_region error.\n", __LINE__);
		goto ERROR2;
	}
	//3.��ʼ��cdev�ṹ
	cdev_init(led_cdev, &button_fops);

	//4.ע���ʼ���õ�cedv�ṹ
	ret = cdev_add(led_cdev, dev_number, 4);
	if(ret < 0){
		printk(KERN_WARNING"%d:cdev_add error.\n", __LINE__);
		goto ERROR3;
	}
	//5.��ȡ���豸��
	major = MAJOR(dev_number);
	printk(KERN_EMERG"major=%d.\n", major);

	//6.����һ����
	led_class = class_create(THIS_MODULE, DEVICE_NAME);
	if(IS_ERR(led_class))
	{
		ret = PTR_ERR(led_class);
		goto ERROR4;
	}
	//7.����һ���豸
	led_device = device_create(led_class, NULL, dev_number, NULL, "%s", DEVICE_NAME);
	if(IS_ERR(led_class))
	{
		ret = PTR_ERR(led_class);
		goto ERROR5;
	}

	printk("�����豸ע�����.major = %d.\n", major);
	return 0; //im
	
	ERROR5:
			class_destroy(led_class);
	ERROR4:
			cdev_del(led_cdev);
	ERROR3: 
			unregister_chrdev_region(dev_number, 4);
	ERROR2:	
			kfree(led_cdev);
	ERROR1: 
			return ret;
}

static void __exit led_exit(void)
{
	//1.ע��cdev�ṹ
	device_destroy(led_class, dev_number);
	
	//2.ע��cdev�ṹ
	class_destroy(led_class);

	//3.ע��cdev�ṹ
	cdev_del(led_cdev);

	//4.�ͷ��豸��
	unregister_chrdev_region(dev_number, 4);

	//5.�ͷ�cdev�ռ�
	kfree(led_cdev);
	
	printk(KERN_EMERG"�����豸ע�����.\n");
}

module_init(led_init);
module_exit(led_exit);
MODULE_LICENSE("GPL");
