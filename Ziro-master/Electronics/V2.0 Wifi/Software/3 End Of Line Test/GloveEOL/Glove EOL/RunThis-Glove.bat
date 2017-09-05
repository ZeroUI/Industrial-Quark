@echo off
set Port=%1
if "%~1"=="" (
	goto UserInp
)
set SerialPort=COM%Port%
:Chk


:ESPCmd
set Exec="%~dp0Files\esptool.exe" -vv -cd ck -cb 115200 -cp %SerialPort% -ca 0x00000 -cf "%~dp0Files\GloveEOL.bin"
Echo %Exec%
wait 3

:loop
%Exec%
goto loop


:UserInp
set /P Port="Invalid COM port Entered. Enter COM Port Number:"
set SerialPort=COM%Port%
goto Chk

:Chk
MODE %SerialPort% | find "RTS" > nul
IF errorlevel 1 goto UserInp
goto ESPCmd

rem :: is a comment symbol in batch file
rem @echo off			:: Turn echo off
rem cls

rem set SerialPort=COM45
rem set FileName=%~nx1
rem set FileAddress=%~f1
rem set FlashSize=%2
rem if "%~2"=="" (
rem 	echo "Setting default Mem Size"
rem 	set FlashSize=2M
rem )

rem set MakeFolder=C:\Users\nikhi\Desktop\ESPmake\MakeFolder
rem set ESPtoolPath=C:\Users\nikhi\AppData\Local\Arduino15\packages\esp8266\tools\esptool\0.4.9/esptool.exe
rem set Port=COM45

rem set Directory=MakeFolder
rem set delim=\
rem set ext=.bin

rem set Compile1="C:\Program Files (x86)\Arduino\arduino-builder" -dump-prefs -logger=machine -hardware "C:\Program Files (x86)\Arduino\hardware" -hardware "C:\Users\nikhi\AppData\Local\Arduino15\packages" -tools "C:\Program Files (x86)\Arduino\tools-builder" -tools "C:\Program Files (x86)\Arduino\hardware\tools\avr" -tools "C:\Users\nikhi\AppData\Local\Arduino15\packages" -built-in-libraries "C:\Program Files (x86)\Arduino\libraries" -libraries "D:\ZeroUI GitHub\libraries" -fqbn=esp8266:esp8266:generic:CpuFrequency=80,FlashFreq=40,FlashMode=dio,UploadSpeed=115200,FlashSize=%FlashSize%,ResetMethod=ck,Debug=Disabled,DebugLevel=None____ -ide-version=10608 -build-path "%MakeFolder%" -warnings=default -prefs=build.warn_data_percentage=75 -verbose "%FileAddress%"
rem set Compile2="C:\Program Files (x86)\Arduino\arduino-builder" -compile -logger=machine -hardware "C:\Program Files (x86)\Arduino\hardware" -hardware "C:\Users\nikhi\AppData\Local\Arduino15\packages" -tools "C:\Program Files (x86)\Arduino\tools-builder" -tools "C:\Program Files (x86)\Arduino\hardware\tools\avr" -tools "C:\Users\nikhi\AppData\Local\Arduino15\packages" -built-in-libraries "C:\Program Files (x86)\Arduino\libraries" -libraries "D:\ZeroUI GitHub\libraries" -fqbn=esp8266:esp8266:generic:CpuFrequency=80,FlashFreq=40,FlashMode=dio,UploadSpeed=115200,FlashSize=%FlashSize%,ResetMethod=ck,Debug=Disabled,DebugLevel=None____ -ide-version=10608 -build-path "%MakeFolder%" -warnings=default -prefs=build.warn_data_percentage=75 -verbose "%FileAddress%"
rem set Compile3=%ESPtoolPath% -vv -cd ck -cb 115200 -cp %SerialPort% -ca 0x00000 -cf %MakeFolder%%delim%%FileName%%ext% 

rem %Compile1% 
rem %Compile2%
rem %Compile3%