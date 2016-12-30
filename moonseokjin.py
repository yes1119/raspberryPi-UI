import RPi.GPIO as GPIO
from Tkinter import *
GPIO.setmode(GPIO.BCM)
import time
import smbus
import spidev
import threading

bus =smbus.SMBus(1)

#Define JoyStick
JOG_LEFT =16
JOG_RIGHT = 20
JOG_UP = 5
JOG_DOWN = 6

count_left = 0
count_right = 0
count_up = 0
count_down = 0

up_image =0
down_image=0
lef_image=0
right_image=0

left_photo_label=0

#Define MPU3208
spi = spidev.SpiDev()
spi.open(0,1)
adc_read =[0,0,0]
all_label=0


#Define LED
LED_1 = 14
LED_2 = 15
led2_img=1
led1_img =1

#define Temp/Humi
addr_temphumi =0x40
cmd_temp=0xf3
cmd_humi=0xf5
soft_reset=0xfe
temp=0.0
humi=0.0
temphumi_label1=0
temphumi_label2=0
temphumi_data = [0,0]

#define Light
addr_light =0x23
con_hr_mode =0x10
light_val=0
light_label1=0
light_data = [0,0]

# Define GPIO to LCD mapping
LCD_RS = 23
LCD_E  = 26 
LCD_D4 = 17
LCD_D5 = 18
LCD_D6 = 27
LCD_D7 = 22

#Define MOTOR
motor_val=1
GPIO_RP =4
GPIO_RN =25
GPIO_EN =12
GPIO.setup(GPIO_RP,GPIO.OUT)
GPIO.setup(GPIO_RN,GPIO.OUT)
GPIO.setup(GPIO_EN,GPIO.OUT)
duty_value=0
p=GPIO.PWM(GPIO_EN,100)

#Defin Piezo
scale=[261,294,329,349,382,440,493,523]
piezo_pin=13
GPIO.setup(piezo_pin,GPIO.OUT)

# Define some device constants
LCD_WIDTH = 16    # Maximum characters per line
LCD_CHR = True
LCD_CMD = False

LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line

# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005

#GPIO SETUP
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)       # Use BCM GPIO numbers
GPIO.setup(LCD_E, GPIO.OUT)  # E
GPIO.setup(LCD_RS, GPIO.OUT) # RS
GPIO.setup(LCD_D4, GPIO.OUT) # DB4
GPIO.setup(LCD_D5, GPIO.OUT) # DB5
GPIO.setup(LCD_D6, GPIO.OUT) # DB6
GPIO.setup(LCD_D7, GPIO.OUT) # DB7
GPIO.setup(LED_1, GPIO.OUT, initial = GPIO.LOW) #LED1
GPIO.setup(LED_2, GPIO.OUT, initial = GPIO.LOW) #LED2

#define FND
addr_fnd=0x20
config_port =0x06
out_port=0x02
seg_data =0

fnd_data = (0xFC,0x60,0xDA,0xF2,0x66,0xB6,0x3E,0xE0,0xFE,0xF6)
digit = (0x7F,0xBF,0xDF,0xEF,0xE7,0xFB)
out_disp=0


################ DEFINE LCD METHOD  #################

def lcd_init():
  # Initialise display
  lcd_byte(0x33,LCD_CMD) # 110011 Initialise
  lcd_byte(0x32,LCD_CMD) # 110010 Initialiseimport threading
  lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
  lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off
  lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
  lcd_byte(0x01,LCD_CMD) # 000001 Clear display
  time.sleep(E_DELAY)


def lcd_toggle_enable():
  # Toggle enable
  time.sleep(E_DELAY)
  GPIO.output(LCD_E, True)
  time.sleep(E_PULSE)
  GPIO.output(LCD_E, False)
  time.sleep(E_DELAY)
  
def lcd_byte(bits, mode):
  # Send byte to data pins
  # bits = data
  # mode = True  for character
  #        False for command

  GPIO.output(LCD_RS, mode) # RS

  # High bits
  GPIO.output(LCD_D4, False)
  GPIO.output(LCD_D5, False)
  GPIO.output(LCD_D6, False)
  GPIO.output(LCD_D7, False)
  if bits&0x10==0x10:
    GPIO.output(LCD_D4, True)
  if bits&0x20==0x20:
    GPIO.output(LCD_D5, True)
  if bits&0x40==0x40:
    GPIO.output(LCD_D6, True)
  if bits&0x80==0x80:
    GPIO.output(LCD_D7, True)

  # Toggle 'Enable' pin
  lcd_toggle_enable()

  # Low bits
  GPIO.output(LCD_D4, False)
  GPIO.output(LCD_D5, False)
  GPIO.output(LCD_D6, False)
  GPIO.output(LCD_D7, False)
  if bits&0x01==0x01:
    GPIO.output(LCD_D4, True)
  if bits&0x02==0x02:
    GPIO.output(LCD_D5, True)
  if bits&0x04==0x04:
    GPIO.output(LCD_D6, True)
  if bits&0x08==0x08:
    GPIO.output(LCD_D7, True)

  # Toggle 'Enable' pin
  lcd_toggle_enable()


def lcd_string(message,line):
  # Send string to display
  message = message.ljust(LCD_WIDTH," ")
  lcd_byte(line, LCD_CMD)
  for i in range(LCD_WIDTH):
    lcd_byte(ord(message[i]),LCD_CHR)

################# EVENT HANDLER #####################

def clear():
  global left_image,right_image,up_image,down_image

  left_image = PhotoImage(file='left_white.png')
  left_photo_label.configure(image =left_image)

  right_image = PhotoImage(file='right_white.png')
  right_photo_label.configure(image =right_image)

  up_image = PhotoImage(file='up_white.png')
  up_photo_label.configure(image =up_image)

  down_image = PhotoImage(file='down_white.png')
  down_photo_label.configure(image =down_image)


    
def handler(channel):
    global left_image,right_image,up_image,down_image
    global left_photo_label
    
    if channel == JOG_LEFT:
       left_image = PhotoImage(file='left_red.png')
       left_photo_label.configure(image =left_image)
       
        
    if channel == JOG_RIGHT:
       right_image = PhotoImage(file='right_red.png')
       right_photo_label.configure(image =right_image)

    if channel == JOG_UP:
       up_image = PhotoImage(file='up_red.png')
       up_photo_label.configure(image =up_image)


    if channel == JOG_DOWN:
       down_image = PhotoImage(file='down_red.png')
       down_photo_label.configure(image =down_image)


    time.sleep(0.1)
    clear()



def led1():
    global led1_img
    GPIO.output(LED_1, not GPIO.input(LED_1))
    if GPIO.input(LED_1) ==1 :
      led1_img = PhotoImage(file='led_on.png')
      led1_photo_label.configure(image =led1_img)

    

    else:
      led1_img = PhotoImage(file='led_off.png')
      led1_photo_label.configure(image =led1_img)


      
def led2():
    global led2_img
    GPIO.output(LED_2, not GPIO.input(LED_2))

    if GPIO.input(LED_2) ==1 :
      led2_img = PhotoImage(file='led_on.png')
      led2_photo_label.configure(image =led2_img)

    else:
      led2_img = PhotoImage(file='led_off.png')
      led2_photo_label.configure(image =led2_img)



def Tlcd_disp_Line1(ev):
    data =TextLCD_Edit1.get()
    lcd_string(data,LCD_LINE_1)
    image_in_label1.configure(text = data)
     
    
def Tlcd_disp_Line2(ev):
    data= TextLCD_Edit2.get()
    lcd_string(data,LCD_LINE_2)
    image_in_label2.configure(text =data)

def seg_thread_start(ev):
    global seg_data,t1
    seg_data= seg_Edit.get()

    t1.start()


class seg_Edit_Show(threading.Thread):
    def run(self):
        global seg_data

        while True:
          seg_value =int(seg_data)

          out_disp=fnd_data[seg_value / 100000] <<8 | digit[0]
          bus.write_word_data(addr_fnd,out_port,out_disp)
          seg_value = seg_value %100000    

          time.sleep(0.002)
          bus.write_word_data(addr_fnd,out_port,0x0000)
          time.sleep(0.0002)

          out_disp=fnd_data[seg_value / 10000] <<8 | digit[1]
          bus.write_word_data(addr_fnd,out_port,out_disp)
          seg_value = seg_value %10000

          time.sleep(0.002)
          bus.write_word_data(addr_fnd,out_port,0x0000)
          time.sleep(0.0002)

          out_disp=fnd_data[seg_value / 1000] <<8 | digit[2]
          bus.write_word_data(addr_fnd,out_port,out_disp)
          seg_value = seg_value %1000

          time.sleep(0.002)
          bus.write_word_data(addr_fnd,out_port,0x0000)
          time.sleep(0.0002)

          out_disp=fnd_data[seg_value / 100] <<8 | digit[3]
          bus.write_word_data(addr_fnd,out_port,out_disp)
          seg_value = seg_value %100

          time.sleep(0.002)
          bus.write_word_data(addr_fnd,out_port,0x0000)
          time.sleep(0.0002)

          out_disp=fnd_data[seg_value / 10] <<8 | digit[4]
          bus.write_word_data(addr_fnd,out_port,out_disp)
          seg_value = seg_value %10

          time.sleep(0.002)
          bus.write_word_data(addr_fnd,out_port,0x0000)
          time.sleep(0.0002)

          out_disp=fnd_data[seg_value] <<8 | digit[5]
          bus.write_word_data(addr_fnd,out_port,out_disp)
          seg_value = seg_value

          time.sleep(0.002)
          bus.write_word_data(addr_fnd,out_port,0x0000)
          time.sleep(0.0002)

     

def piezo_play():
    p=GPIO.PWM(piezo_pin,100)
    p.start(10)

    for i in range(8):
            p.ChangeFrequency(scale[i])
            time.sleep(0.5)

    p.stop()


def temphumi_start():
    global t2
    t2.start()



class temphumi_show(threading.Thread):
  def run(self):
      while 1:
        global temp,humi,temphumi_data,cmd_temp,cmd_humi,addr_temphumi

        #temp
        bus.write_byte(addr_temphumi,cmd_temp)
        time.sleep(0.260)

        for i in range(0,2,1):
             temphumi_data[i] = bus.read_byte(addr_temphumi)

             val= temphumi_data[0] << 8 |temphumi_data[1]
             temp = -46.85 + 175.72/65536*val
             

        temphumi_label1.configure(text = 'temp: ' + str(float(round(temp,2))))
            

        #humi
        bus.write_byte(addr_temphumi,cmd_temp)
        time.sleep(0.260)

        for i in range(0,2,1):
            temphumi_data[i] = bus.read_byte(addr_temphumi)

            val= temphumi_data[0] << 8 |temphumi_data[1]
            humi = -6.0+125.0/65536*val

        temphumi_label2.configure(text = 'humi: ' + str(float(round(humi,2))))

def light_start():
    global t3
    t3.start()

class light_show(threading.Thread):
  def run(self):
      while 1:
         global light_val,light_label1

         light_data = bus.read_i2c_block_data(addr_light,con_hr_mode)
         time.sleep(0.2)
         light_val =((light_data[1] + (256 * light_data[0])) /1.2)

         light_label1.configure(text = 'light: ' + str(float(round(light_val,2))))
         
    


def change_speed(ev):
    global duty_value
    duty_value= motor_speed.get()
    

    if duty_value ==0:
       GPIO.output(GPIO_EN,False)
       GPIO.output(GPIO_RN,False)
       GPIO.output(GPIO_RP,False)

    p.start(duty_value)


def motor_left():
    global duty_value

    GPIO.output(GPIO_RP,False)
    GPIO.output(GPIO_RN,True)
    p.start(duty_value)


def motor_right():
    global duty_value


    GPIO.output(GPIO_RP,True)
    GPIO.output(GPIO_RN,False)
    p.start(duty_value)


def analog_read(ch):
    #12 bit
    r= spi.xfer2([0x6 | (ch & 0x7) >>2, ((ch & 0x07) <<6),0])
    adcout =((r[1] & 0xf) << 8) + r[2]

    return adcout    

def display_all():
    global all_label,adc_read
    

    adc_read[0] = analog_read(0)
    adc_read[1] = analog_read(1)
    adc_read[2] = analog_read(2)


    all_label.configure( text='cds / vr / sound : ' + str(adc_read))


###################### INIT ###################
root = Tk()
root.title("TK LED")
root.geometry("1280x720+100+100")
lcd_init()
bus.write_word_data(addr_fnd,config_port,0x0000)
bus.write_byte(addr_temphumi,0x07) #reset
bus.write_byte(addr_light,0x07)

for i in [JOG_LEFT,JOG_RIGHT,JOG_UP,JOG_DOWN]:
    GPIO.setup(i,GPIO.IN)
    GPIO.add_event_detect(i,GPIO.RISING,callback=handler)

t1=seg_Edit_Show()
t2=temphumi_show()
t3=light_show()

################### UI Set #######################

#TEXTLCD 
img = PhotoImage(file='lcd.png')
TextLCD_photo_label = Label(image=img)
TextLCD_photo_label.image = img
TextLCD_photo_label.place(x=10, y=10)

image_in_label1 = Label(root, text='                                                            ',bg='#7FFF00')
image_in_label1.place(x=50, y=105)

image_in_label2 = Label(root, text='                                                            ',bg='#7FFF00')
image_in_label2.place(x=50, y=135)

TextLCD_label1 = Label(root, text='Line 1:')
TextLCD_label1.place(x=10, y=250)

TextLCD_label2 = Label(root, text='Line 2:')
TextLCD_label2.place(x=10, y=270)

TextLCD_Edit1 = Entry(root)
TextLCD_Edit1.place(x=60, y=250)
TextLCD_Edit1.bind('<Return>',Tlcd_disp_Line1)

TextLCD_Edit2 = Entry(root)
TextLCD_Edit2.place(x=60, y=270)
TextLCD_Edit2.bind('<Return>',Tlcd_disp_Line2)


#7seg
seg_img = PhotoImage(file='seg.png')
seg_photo_label = Label(image=seg_img)
seg_photo_label.image = seg_img
seg_photo_label.place(x=10, y=300)

seg_label1 = Label(root, text='Input Data')
seg_label1.place(x=20, y=370)

seg_label2 = Label(root, text='(0~999999)')
seg_label2.place(x=20, y=390)

seg_Edit= Entry(root, width = 6)
seg_Edit.place(x=120, y=370)
seg_Edit.bind('<Return>',seg_thread_start)

#PIEZO
piezo_img = PhotoImage(file='piezo.png')
piezo_photo_label = Label(image=piezo_img)
piezo_photo_label.image = piezo_img
piezo_photo_label.place(x=200, y=420)

piezo_button =Button(root,text='PIEZ0 PLAY',command =piezo_play)
piezo_button.place(x=200, y=500)

#LED

led1_img = PhotoImage(file='led_off.png')
led1_photo_label = Label(image=led1_img)
led1_photo_label.image = led1_img
led1_photo_label.place(x=10, y=420)


led2_img = PhotoImage(file='led_off.png')
led2_photo_label = Label(image=led2_img)
led2_photo_label.image = led2_img
led2_photo_label.place(x=90, y=420)

led1_button =Button(root,text='LED1 ON',command =led1)
led1_button.place(x=10, y=500)


led2_button =Button(root,text='LED2 ON',command =led2)
led2_button.place(x=90, y=500)

#TEMP/HUMI

temphumi_img = PhotoImage(file='temphumi.png')
temphumi_photo_label = Label(image=temphumi_img)
temphumi_photo_label.image = temphumi_img
temphumi_photo_label.place(x=10, y=550)

temphumi_button =Button(root,text='TEMPHUMI\nDISPLAY',command =temphumi_start)
temphumi_button.place(x=10, y=630)

temphumi_label1 = Label(root, text='temp:0.00')
temphumi_label1.place(x=10, y=670)

temphumi_label2 = Label(root, text='humi:0.00')
temphumi_label2.place(x=10, y=690)

#LIGHT

light_img = PhotoImage(file='light.png')
light_photo_label = Label(image=light_img)
light_photo_label.image = light_img
light_photo_label.place(x=200, y=550)

light_button =Button(root,text='LIGHT\nDISPLAY',command =light_start)
light_button.place(x=200, y=630)

light_label1 = Label(root, text='light:0.00')
light_label1.place(x=200, y=670)


#MOTOR

motor_img = PhotoImage(file='motor.png')
motor_photo_label = Label(image=motor_img)
motor_photo_label.image = motor_img
motor_photo_label.place(x=350, y=10)

motor_button =Button(root,text='MOTOR\nLeft\nOn',command =motor_left)
motor_button.place(x=350, y=200)

motor_speed = Scale(root,label='MOTOR SPEED',orient = 'h',command =change_speed)
motor_speed.place(x=500,y=200)

motor_button =Button(root,text='MOTOR\nRight\nOn',command =motor_right)
motor_button.place(x=680, y=200)


#MPU3208

cds_img = PhotoImage(file='cds.png')
cds_photo_label = Label(image=cds_img)
cds_photo_label.image = cds_img
cds_photo_label.place(x=350, y=270)

vr_img = PhotoImage(file='vr.png')
vr_photo_label = Label(image=vr_img)
vr_photo_label.image = vr_img
vr_photo_label.place(x=500, y=270)

sound_img = PhotoImage(file='sound.png')
sound_photo_label = Label(image=sound_img)
sound_photo_label.image = sound_img
sound_photo_label.place(x=670, y=270)

motor_button =Button(root,text='                      Display All                      '
                     ,command =display_all)
motor_button.place(x=430, y=350)

all_label = Label(root, text='cds / vr / sound : 0 / 0 / 0')
all_label.place(x=350, y=380)

#backImage
back_img = PhotoImage(file='raspberry.png')
back_photo_label = Label(image=back_img)
back_photo_label.image = back_img
back_photo_label.place(x=800, y=10)


#JoyStick


left_img = PhotoImage(file='left_white.png')
left_photo_label= Label(image=left_img)
left_photo_label.image = left_img
left_photo_label.place(x=400, y=520)

down_img = PhotoImage(file='down_white.png')
down_photo_label= Label(image=down_img)
down_photo_label.image = down_img
down_photo_label.place(x=520, y=630)

right_img = PhotoImage(file='right_white.png')
right_photo_label= Label(image=right_img)
right_photo_label.image = right_img
right_photo_label.place(x=630, y=520)

up_img = PhotoImage(file='up_white.png')
up_photo_label= Label(image=up_img)
up_photo_label.image = up_img
up_photo_label.place(x=520, y=420)

stick_img = PhotoImage(file='joystick.png')
stick_photo_label = Label(image=stick_img)
stick_photo_label.image = stick_img
stick_photo_label.place(x=470, y=480)


try:
    root.mainloop()

finally:

    print "Clean"
    GPIO.cleanup()
    spi.close()
    bus.close
