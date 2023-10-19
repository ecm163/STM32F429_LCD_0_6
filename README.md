
# Example0_FreeRTOS_STM32F29Discovery_STM32CubeIDE

# **Summary:**

  This project runs on a basic FreeRTOS example, designed using STM32CubeIDE running on the STM32F29-Discovery board, which executes 3 tasks:
  
  **1-** Shows the execution time on the STM32F429-Discovery display (in hours, minutes and seconds).
  
  **2-** It turns on 4 LEDs sequentially, turns them off, the sequence of turning on the 4 LEDs again, turns them off, and so on.
  
  **3-** It takes an analog signal from a potentiometer, and sends it through a UART-USB converter to the PC. The values in volts of the potentiometer can be displayed on your PC using 
  serial terminal software (such as Realterm, which is the one I use).
  
  **Below I put a link to a video so you can see how it works:**
  
  https://www.youtube.com/shorts/trmJfeHAia4
  
  Below (at the end) I show how you can run the program correctly on your PC.

# **Required software:**

  - STM32CubeIDE.
  - Some serial terminal software (for example, Realterm, which is what I use).
  
# **Required hardware:**

  **1-**	STM32F429-Discovery board:
  
  [![Imagen1.jpg](https://i.postimg.cc/sXgv46M7/Imagen1.jpg)](https://postimg.cc/CZ9hF4Hx)
  
  **2-** UART-USB converter hardware (CP2102, FTDI…etc):
  
  [![Imagen2.jpg](https://i.postimg.cc/tgKVptJx/Imagen2.jpg)](https://postimg.cc/rd9p94SV)
  
  **3-** Breadboard:
  
  [![Imagen3.jpg](https://i.postimg.cc/gkLF6V2J/Imagen3.jpg)](https://postimg.cc/Cz063fDT)
  
  **4-** Dupont type cables (male-female, female-female):
  
  [![Imagen4.jpg](https://i.postimg.cc/x1R200dy/Imagen4.jpg)](https://postimg.cc/hXfwsRTf)
  
  **5-** 4 LEDs for breadboard:
  
  [![Imagen5.jpg](https://i.postimg.cc/65cjqTd3/Imagen5.jpg)](https://postimg.cc/jLWQkx30)
   
  **6-** 4 resistors 220 ohms:
  
  [![Imagen6.jpg](https://i.postimg.cc/1tK6BVRV/Imagen6.jpg)](https://postimg.cc/jwLLd57R)
  
  **7-** A potentiometer for a breadboard (there are different types that can be inserted into a breadboard):
  
  [![Imagen7.jpg](https://i.postimg.cc/ydr677vr/Imagen7.jpg)](https://postimg.cc/CBkpNpyj) 

# **Connected:**

[![Imagen8.jpg](https://i.postimg.cc/WzLNd5cM/Imagen8.jpg)](https://postimg.cc/jwc09Q55)

# **Steps to successfully run the program on your PC:**

I am going to explain it for Windows assuming that in MacOS or Linux it will be similar:

If you download the file to your PC, and then unzip it, **delete "-main"**.

[![Descarga1.jpg](https://i.postimg.cc/4NCQkc79/Descarga1.jpg)](https://postimg.cc/NL4HR5xs)

[![Descarga2.jpg](https://i.postimg.cc/rwMbT30x/Descarga2.jpg)](https://postimg.cc/6ybzLHhp)

[![Descarga3.jpg](https://i.postimg.cc/mgmdsDGW/Descarga3.jpg)](https://postimg.cc/LJZkkmXx)

[![Descarga4.jpg](https://i.postimg.cc/1R0rStp7/Descarga4.jpg)](https://postimg.cc/bdJtHqV1)

Then follow the following steps

**1-** Navigate your PC to the following path:

C:\Users\yourUserName\STM32Cube\Repository 

[![Imagen9.jpg](https://i.postimg.cc/ydTQRNwv/Imagen9.jpg)](https://postimg.cc/G8Hxwb3y)

**2-** Make sure the folder is there:

**STM32Cube_FW_F4_V1.27.1** 

[![Imagen10.jpg](https://i.postimg.cc/23N1fz5N/Imagen10.jpg)](https://postimg.cc/QKS8QZ5f)

My “yourUserName” is Emilio.

**3-** If this folder is not there, copy the folder:

**STM32Cube_FW_F4_V1.27.1**

In the path already indicated:

C:\Users\yourUserName\STM32Cube\Repository

In my case the path is:

C:\Users\Emilio\STM32Cube\Repository

To do this, follow the following steps: 

[![Imagen11.jpg](https://i.postimg.cc/52xBrMkF/Imagen11.jpg)](https://postimg.cc/MvFfcgwW)

[![Imagen12.jpg](https://i.postimg.cc/4xNc5Qm6/Imagen12.jpg)](https://postimg.cc/8j9skWdc)

[![Imagen13.jpg](https://i.postimg.cc/tCmqK3Bt/Imagen13.jpg)](https://postimg.cc/LnLFf1Kn)

[![Imagen14.jpg](https://i.postimg.cc/sDtFPwn8/Imagen14.jpg)](https://postimg.cc/0bdHPd1d)

**4-** Next, there will be the STM32CubeIDE and import the project into your workspace:

[![Imagen15.jpg](https://i.postimg.cc/PrM3JygY/Imagen15.jpg)](https://postimg.cc/gnnqBVk2)

**5-** Right click on the project name and a drop-down menu will open:

[![Imagen16.jpg](https://i.postimg.cc/rwmJmkC2/Imagen16.jpg)](https://postimg.cc/rKBWn7Pf)

**6-** Click on “Properties”:

[![Imagen17.jpg](https://i.postimg.cc/W1hGV7T2/Imagen17.jpg)](https://postimg.cc/pmMhYzm7)

**7-** A window appears:

[![Imagen17-1.jpg](https://i.postimg.cc/wxFRBbqh/Imagen17-1.jpg)](https://postimg.cc/0rMyWnJQ)

**8-** Click on the “C/C++ General” drop-down menu:

[![Imagen18.jpg](https://i.postimg.cc/gcyFyV9k/Imagen18.jpg)](https://postimg.cc/Jth2MB49)

**9-** Click on “Path and Symbols”:

[![Imagen19.jpg](https://i.postimg.cc/FH4xyS4m/Imagen19.jpg)](https://postimg.cc/JGpJmG0d)

**10-** In the window that opens make sure you are in the “Includes” tab:

[![Imagen20.jpg](https://i.postimg.cc/9MY2fZCM/Imagen20.jpg)](https://postimg.cc/ygxtQJJ4)
 
**11-** In the “Include directories” there are paths with the form:

C:/Users/Emilio/STM32Cube…

**12-** If you want it to work, change “Emilio” to your username. For example, if your name is Julian, change “Emilio” to “Julian.” To do this, select each path and edit it as follows:
 
[![Imagen21.jpg](https://i.postimg.cc/SR4W60Bd/Imagen21.jpg)](https://postimg.cc/R6P6mYVJ)

**13-** Change “Emilio” to your username (for example, “Julian”). 

[![Imagen22.jpg](https://i.postimg.cc/PxNtSKDw/Imagen22.jpg)](https://postimg.cc/bDccJx3y)

[![Imagen23.jpg](https://i.postimg.cc/4Nc0RMR1/Imagen23.jpg)](https://postimg.cc/zVJ7nxBL)

[![Imagen24.jpg](https://i.postimg.cc/hPGGCLFg/Imagen24.jpg)](https://postimg.cc/Btdsb1Qw)

**14-** Finally, click on “OK”, and then click on “Apply and Close”.

