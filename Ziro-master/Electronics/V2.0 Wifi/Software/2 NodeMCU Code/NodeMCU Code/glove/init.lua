-- Handimate Glove
-- Set up GPIO
r=0
g=0
b=0
pin_red=2
pin_green=1
pin_but=3
pin_blue=4
gpio.mode(pin_but, gpio.INPUT, gpio.PULLUP)
pwm.setup(pin_red, 100, 512)
pwm.setup(pin_green,100,512)
pwm.setup(pin_blue,100, 0)
pwm.start(pin_red)
pwm.start(pin_green)
pwm.start(pin_blue)
tmr.delay(200000)
pwm.setduty(pin_red,0)
pwm.setduty(pin_green,0)
pwm.setduty(pin_blue,0)

-- Set up WiFi
wifi.setmode(wifi.SOFTAP)
cfg={}
cfg.ssid="Handimate"
cfg.pwd="Handimate"
wifi.ap.config(cfg)

-- Initialize Client Counter
ClientI=-1
ClientSkt={}
ClientId={}

-- Webpage Related
fileName = 'Handimate.htm'

-- Setup TCP Server
ServHTML=0
if srv then
  srv:close()
end
srv=net.createServer(net.TCP, 28800)
srv:listen(80,function(conn) 

  conn:on("receive", function(conn,payload)
    local isOpen=false
    conn:on("sent", function(conn)
    if ServHTML==1 then  
      if not isOpen then
        isOpen=true
        file.open(fileName, 'r')
      end
      local data=file.read(1024)
      if data then
        conn:send(data)
      else
        file.close()
        conn:close()
        conn=nil
      end
    end
    end)

    if string.sub(payload,1,4)=="ESL:" then
      -- Save Client  Socket on New Connection Identifier
      ServHTML=0
      print("Client:NewConn")
      ClientI=ClientI+1
      ClientSkt[ClientI]=conn  
      ClientId[ClientI]=string.sub(payload,5,6)
      return nil
      
    elseif string.sub(payload, 1, 6) == 'GET / ' then
      ServHTML=1  
      print("Browser:homepage")
      conn:send("HTTP/1.1 200 OK\r\n")
      conn:send("Content-type: text/html\r\n")
      conn:send("Connection: close\r\n\r\n")
    elseif string.sub(payload, 1, 8) == 'GET /ja ' then
      ServHTML=1
      print("Browser:Update")
      conn:send("HTTP/1.1 200 OK\r\n")
      conn:send("Content-type: application/json\r\n")
      conn:send("Connection: close\r\n\r\n")
      conn:send("{\"adc\":" .. adc.read(0) .. ",\"but\":" .. (1-gpio.read(pin_but)) .. ",\"red\":" .. r .. ",\"green\":" .. g .. ",\"blue\":" .. b .. "}")
      conn:close()
      conn=nil
      ServHTML=0
    elseif string.sub(payload, 1, 8) == 'GET /cc?' then
      ServHTML=1
      print("Browser:Color Change")
      conn:close()
      conn=nil
      ServHTML=0
        
      local val=string.match(payload,"r=(%d+)")
      if val then
        r=tonumber(val)
      end
      local val=string.match(payload,"g=(%d+)")
      if val then
        g=tonumber(val)
      end
      local val=string.match(payload,"b=(%d+)")
      if val then
        b=tonumber(val)
      end       
      pwm.setduty(pin_red, r)
      pwm.setduty(pin_green, g)
      pwm.setduty(pin_blue, b)
    else
      conn:close()
      ServHTML=0
    end
  end) 
end)

-- Prepare Data
t=require("cmd")
DataStream={} --Contains all Resource Values
DataStream[0]=0

-- Send Data Periodically
tmr.alarm(1,100,1,function()
     if ServHTML==1 then
        return
     end

     -- Read ADC
     local A=adc.read(0)
     A=2*A - 710
     if A<0 then A=1 end
     if A>1000 then A=1000 end
     pwm.setduty(pin_blue, A) -- set GLOVE LED
     DataStream["A"]=tostring(A)
     
     --Read GPIO
     local G=(1-gpio.read(pin_but))
     DataStream["G"]=tostring(G*1000)
     
     --Read I2C Z-axis
     local accZ=0
     DataStream["accZ"]=tostring(accZ)
     
     --Form Data Packet
     local DATA="ESE"
     
     local resource=""
     for k,ID in pairs(ClientId) do
        if ID==cmdID[ID] then   -- Check if ID prescribed
            DATA=DATA..":"..ID
            for index,stream in pairs(cmdDataStream[ID]) do
                --print("Resource:"..tostring(cmdResource[ID][index]).."   StreamReq:"..tostring(stream).."    Val:"..DataStream[stream])
                resource=tostring(cmdResource[ID][index]) 
                DATA=DATA.."-"..resource
                DATA=DATA..tostring(DataStream[stream])
            end
        end
     end   

     DATA=DATA..":"
     -- MultiCast To All Connected SLaves
     for key,val in pairs(ClientSkt) do
        if val~=nil then
                val:send(DATA)
        end
     end
     val=nil
     DATA=nil
end)

-- Print Remaining RAM
print("Heap size: " .. node.heap())   
