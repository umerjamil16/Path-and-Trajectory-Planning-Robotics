function q = inv_kine(x,y,z)
d1 = 5;
q(1) = atan2(y,-x);
q(2) = z - d1;
q(3) = sqrt(x^2 + y^2);
end