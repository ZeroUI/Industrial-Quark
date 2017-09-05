pwr_en=0
gpio.mode(pwr_en,gpio.OUTPUT);
gpio.write(pwr_en,1);

pwr_sw=7
gpio.mode(pwr_sw,gpio.INT,gpio.PULLUP);

tmr.delay(1000);



--gpio.trig(pwr_sw,"down",function()
  --  gpio.mode(pwr_sw,gpio.INPUT);
  --  print("det");
  --  tmr.delay(2000);
  --  if(gpio.read(pwr_sw)==0) then
  --      gpio.mode(pwr_sw,gpio.OUTPUT);
  --      gpio.write(pwr_sw,0);
  --      gpio.write(pwr_en,0);
  --      tmr.delay(1000);
  --  else
  --      gpio.mode(pwr_sw,gpio.INT,gpio.PULLUP);
  --  end
--end)

gpio.trig(pwr_sw,"down",function()
    gpio.mode(pwr_sw,gpio.OUTPUT);
    gpio.write(pwr_en,0);
    gpio.write(pwr_sw,0);
end)

tmr.alarm(4,100,tmr.ALARM_AUTO,function()
  gpio.mode(7,gpio.INPUT);
    local a=gpio.read(7);
   print(a);
   gpio.mode(7,gpio.INT);
end)
