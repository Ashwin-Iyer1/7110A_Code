# %%
import pyautogui
from time import sleep
import easyocr


# %%
if(pyautogui.locateOnScreen('Start.png') is None):
    print("Start button not found")
else:
    Start = pyautogui.locateOnScreen('Start.png')
if(pyautogui.locateOnScreen('Load.png') is None):
    print("Load button not found")
else:
    Load = pyautogui.locateOnScreen('Load.png')
if(pyautogui.locateOnScreen('0.png') is None):
    print("Cannot find score")
else:
    Score = pyautogui.locateOnScreen('0.png')
if(pyautogui.locateOnScreen('Reload.png') is None):
    print("Reload button not found")
else:
    Reload = pyautogui.locateOnScreen('Reload.png')



# %%
def first_Balls():
    pyautogui.click(Reload)
    sleep(5)
    pyautogui.click(Start)
    sleep(1)
    if(pyautogui.locateOnScreen('Stop.png') is None):
        print("Stop button not found")
        pyautogui.click(Start)
    sleep(22)
    pyautogui.screenshot('screenshot.png', region=(Score))
    score = easyocr.Reader(['en']).readtext('screenshot.png', detail = 0)
    print(score[0])
    if(int(score[0]) < 29):
        pyautogui.click(Start)
        print("Points not high enough, running again")
        sleep(.5)
        if(pyautogui.locateOnScreen('Submit.png') is None):
            print("Submit button not found")
        else:
            Submit = pyautogui.locateOnScreen('Submit.png')
        if(pyautogui.locateOnScreen('Restart.png') is None):
            print("Restart button not found")
        else:
            Restart = pyautogui.locateOnScreen('Restart.png')
        pyautogui.click(Restart)
        sleep(2)
        first_Balls()
    else:
        second_Balls()


def second_Balls():
    for i in range(0, 36):
        sleep(1)
        pyautogui.click(Load)
    if(pyautogui.locateOnScreen('Submit.png') is None):
        print("Submit button not found")
    else:
        Submit = pyautogui.locateOnScreen('Submit.png')
    if(pyautogui.locateOnScreen('Restart.png') is None):
        print("Restart button not found")
    else:
        Restart = pyautogui.locateOnScreen('Restart.png')
    sleep(2)
    pyautogui.click(Submit)
    sleep(2)
    pyautogui.click(Restart)
    sleep(2)
    first_Balls()


first_Balls()