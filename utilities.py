import inspect
import numpy

# Displays text print along with script caller for better debuging.
def log(text):
    logCaller = inspect.stack()[1][1] #Inspect stack to see what script path called this
    logCaller = logCaller.split('/') #Split path
    logCaller = logCaller[len(logCaller) - 1] #Select last item to get script name
    print("[%s] %s" % (logCaller, text)) # Print '[caller] text'
