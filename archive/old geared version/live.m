t = 0 ;
x = 0 ;
startSpot = 0;
interv = 500 ; % considering 1000 samples
step = 1 ; % lowering step has a number of cycles and then acquire more data
while ( t <interv )
    b = sin(t)+5;
    x = [ x, b ]
    line1 = plot(x) ;
    xlabel('x')
    line1_label = "sin(x)+5"
    legend(line1,line1_label)
    if ((t/step)-500 < 0)
      startSpot = 0;
    else
      startSpot = (t/step)-500;
    end
    axis([ startSpot, (t/step+50), 0 , 10 ]);
    grid
    t = t + step;
    drawnow limitrate;
    pause(0.01)
  end
