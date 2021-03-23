import base64
with open("target_00.bmp","rb") as imgfile:
    mystring = base64.b64encode(imgfile.read())
print(mystring)
