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

# %%
found = False
found2 = False
def first_Balls():
    pyautogui.click(Start)
    sleep(20)
    pyautogui.screenshot('screenshot.png', region=(Score))
    score = easyocr.Reader(['en']).readtext('screenshot.png', detail = 0)
    print(score[0])
    if(score[0] < 30):
        pyautogui.click(Start)
        print("Points not high enough, running again")
        sleep(2)
        first_Balls()
    else:
        second_Balls()


def second_Balls():
    for i in range(0, 40):
        sleep(1)
        pyautogui.click(Load)
    while(found != True and found2 != True):
        if(pyautogui.locateOnScreen('Submit.png') is None):
            print("Submit button not found")
        else:
            found = True
            Submit = pyautogui.locateOnScreen('Submit.png')
        if(pyautogui.locateOnScreen('Restart.png') is None):
            print("Restart button not found")
        else:
            found2 = True
            Restart = pyautogui.locateOnScreen('Restart.png')
    sleep(2)
    pyautogui.click(Submit)
    sleep(2)
    pyautogui.click(Restart)
    sleep(2)
    first_Balls()



first_Balls()