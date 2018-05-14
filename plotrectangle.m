function plotrectangle(A,B,C,D,color)
x = [A(1);B(1);C(1);D(1);A(1)];
y = [A(2);B(2);C(2);D(2);A(2)];
z = [A(3);B(3);C(3);D(3);A(3)];
h = plot3(x,y,z,color);
set(h,'linewidth',2);
end