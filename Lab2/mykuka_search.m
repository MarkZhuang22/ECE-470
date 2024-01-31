function structure = mykuka_search(delta)
    
    DH = [0     400.0    25.0       pi/2;
      0     0      315.0         0;
      0     0      35.0       pi/2;
      0     365.0    0       -pi/2;
      0     0      0        pi/2;
      0     161.44+delta(2) -296.23+delta(1)    0];
  
    structure = SerialLink(DH);
end