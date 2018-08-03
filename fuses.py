from os.path import join
from SCons.Script import DefaultEnvironment

env = DefaultEnvironment()

#env.Replace(UPLOADHEXCMD="$UPLOADER $UPLOADERFLAGS -U flash:w:$SOURCES:i")
env.Replace(
    MYUPLOADERFLAGS=[
        "-v",
        "-p", "$BOARD_MCU",
        "-P", "usb",
        "-C",
        '"%s"' % join("$PIOPACKAGES_DIR", "tool-avrdude", "avrdude.conf"),
        "-c", "$UPLOAD_PROTOCOL",
        "-b", "$UPLOAD_SPEED"
     ],
    UPLOADHEXCMD='"$UPLOADER" $MYUPLOADERFLAGS -U flash:w:$SOURCES:i'
    )

env.Replace(FUSESCMD="avrdude $UPLOADERFLAGS -U lock:w:0xFF:m -U lfuse:w:0x62:m -U hfuse:w:0xD9:m -U efuse:w:0xFF:m")

# uncomment line below to see environment variables
print " ********************** fuses328p.py ***********************"
print env["UPLOADHEXCMD"]
print env["FUSESCMD"]
#print "-"
#print env.Dump()
print " ********************** fuses328p.py ***********************"
