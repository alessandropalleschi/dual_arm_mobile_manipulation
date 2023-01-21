function generate_box_stl(x,y,z)

    vert = [x/2,    x/2,    x/2,    x/2,    -x/2,   -x/2,   -x/2,   -x/2; ...
            y/2,    y/2,    -y/2    -y/2,   y/2,    y/2,    -y/2    -y/2; ...
            z/2,    -z/2,   z/2,    -z/2,   z/2,    -z/2,   z/2,    -z/2]';
    
    f = [2 6 8; 2 4 8; 
         1 5 7; 1 3 7; 
         2 1 4; 4 1 3; 
         6 7 5; 6 8 7;
         2 6 5; 2 1 5;
         4 8 7; 4 3 7];
    
    stlwrite('flange.stl',f,vert)
end