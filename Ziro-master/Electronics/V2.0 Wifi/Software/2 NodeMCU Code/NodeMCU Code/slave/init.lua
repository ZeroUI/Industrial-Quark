-- SLAVE Address
ID="00"
Resources={"MA","LG","LR","LB"}

-- Set up WiFi
wifi.setmode(wifi.STATION) 
wifi.sta.config("Handimate","Handimate")
wifi.sta.connect()

-- Set up GPIO pins
r=0
g=0
b=0
pin_red=2
pin_green=1
pin_but=3
ServMotPWM=4
gpio.mode(pin_but, gpio.INPUT, gpio.PULLUP)
pwm.setup(pin_red, 100, 512)
pwm.setup(pin_green,100,512)
pwm.setup(ServMotPWM,100,0)
pwm.start(pin_red)
pwm.start(pin_green)
pwm.start(ServMotPWM)
tmr.delay(200000)
pwm.setduty(pin_red,0)
pwm.setduty(pin_green,0)
pwm.setduty(ServMotPWM,0)

sk=nil
tmr.alarm(0, 1000, 1, function()
if (wifi.sta.getip() ~= nil) then  
    if(sk==nil) then      
        sk = net.createConnection(net.TCP,  0) 
        sk:connect(80, "192.168.4.1") -- server ESP IP
        sk:on("connection", OnConnectionResp )
    end    
    sk:on("receive", OnClientRsk)    
else
   print("Connecting")
end
collectgarbage() 
end)

function OnConnectionResp(sk)  
    tmr.delay(10000)
    sk:send("ESL:"..ID) 
end





ResourceFunc={}

-- Green LED LG
ResourceFunc=
{
LG=function(val)
    if val==nil then val=0 end
    val=tonumber(val)
    if(val<0) then val=0
    elseif(val>1000) then val=1000
    end
    pwm.setduty(pin_green,val)
end,


-- RED LED LR
LR=function(val)
    if val==nil then val=0 end
    val=tonumber(val)
    if(val<0) then val=0
    elseif(val>1000) then val=1000
    end
    pwm.setduty(pin_red,val)
end,

            
-- Motor PWM
MA=function(val)
    if val==nil then val=0 end
    val=tonumber(val)
    if val>214 then val=214
    elseif val<48 then val=48
    end
    pwm.setduty(ServMotPWM,val)
end
}







function OnClientRsk(sk,payload)
        print(payload)
        local i=0 local j=0 local k=0 local len=0 local temp=0
        if string.sub(payload,1,4)=="ESE:" then
            -- Begin Parsing Message
            i,j=string.find(payload,":"..ID)
            len=string.find(payload,":",j) 
            if len==nil then len=string.len(payload)
            else             len=len-1 end
            
            -- Check if Message present
            if (i~=nil) then
                -- Message present
                for key,res in pairs(Resources) do
                    j=string.find(payload,"-"..res,i)--Find parameters

                    if (j~=nil) then
                        k=string.find(payload,"-",j+1)
                        if(k==nil) then k=len
                        elseif(k>len) then k=len
                        else k=k-1  end
                        
                        temp=string.sub(payload,j+3,k)
                        if res=="MA" then ResourceFunc.MA(temp)

                        elseif res=="LR" then ResourceFunc.LR(temp)

                        elseif res=="LG" then ResourceFunc.LG(temp)
                        end
                    end
                end

            end
            
        end
end