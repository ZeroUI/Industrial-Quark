--I2C--
id=0
sda=5
scl=6  
i2c.setup(id,sda,scl,i2c.SLOW)
devAddr=0x68
function i2cWriteByte(id,regAddr,data)
    i2c.start(id)
    if(i2c.address(id,devAddr,i2c.TRANSMITTER)) then
        if data==nil then
            len = i2c.write(id,regAddr)
        else
            len = i2c.write(id,regAddr,data)
        end
        --print("Writing "..len.."  bytes")  
    end 
    i2c.stop(id)
    return len
end
function i2cReadByte(id,bytesToRead)
    i2c.start(id)
    if(i2c.address(id,devAddr,i2c.RECEIVER)) then
        dataRead = i2c.read(id,bytesToRead)  
    end
    i2c.stop(id) 
    return dataRead
end
function read_reg(regAddr)      
    i2cWriteByte(id,regAddr)
    dataRead=i2cReadByte(id,1)
    --print("SingleByteRead from address(in dec) "..tostring(regAddr))
    return dataRead
end

function i2cInit()    
    len=i2cWriteByte(id,0x2D,0x08)
    if len>0 then
        print("Initializing I2C done")
    else
        print("I2C error in initializing")
    end
end
    
function readAccZ0() 
        readAccZval=string.byte(read_reg(0x36))
        if (readAccZval== nil) then
            print("ERROR:Register reading nil")
        end
    return readAccZval
end

function calibZ()
    readAccZval=readAccZ0()
	--print(readAccZval)
	angleZ= 490 -(2*readAccZval)
    if(angleZ>180) then
        angleZ=180
    elseif(angleZ<0) then
        angleZ=0
    end
	--print("angleZ "..tostring(angleZ))
    return angleZ    
end

--sample code for I2C lib
i2cInit()
print("Checking DevID(229) is.."..string.byte(read_reg(0x00)))
print("Caliberated Z Accel between 0 to 180")
angleZ=calibZ()
print("angleZ "..tostring(angleZ))
