function disp_description()
disp("joints:");
disp("  1)  world to foot  (fixed)");
disp("  2)  foot to invl1  (fixed)");
disp("  3)  invl1 to invl2 (rev)");
disp("  4)  invl2 to leg1  (rev)");
disp("  5)  leg1 to invl3  (fixed)");
disp("  6)  invl3 to leg2  (rev)");
disp("  7)  leg2 to invl4  (fixed)");
disp("  8)  invl4 to invl5 (rev)");
disp("  9)  invl5 to invl6 (rev)");
disp("  10) invl6 to head  (fixed)");
disp("Usable variables: m(1,10)   I(3,10)   dim(2,10)   joint_to_com(3,10)   rot(3,10)");
end