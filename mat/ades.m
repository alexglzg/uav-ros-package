function aD = ades(P,focal,z,psi)
%P is a 3x4 matrix with the initial XYZ coord of the POI from the target w.r.t. the
%fixed frame.
%focal is the camera's focal length.
%z is the desired depth of the UAV.
%psi is the desired yaw angle of the UAV. Normally, zero

R_psi = [cos(psi),-sin(psi) 0; 
         sin(psi), cos(psi), 0; 
         0, 0, 1];  

xyz_vir_coord = transpose(R_psi)*P;

x_vir = xyz_vir_coord(1,:);
y_vir = xyz_vir_coord(2,:);

u_vir_cam = (focal/z)*x_vir;
n_vir_cam = (focal/z)*y_vir;

ug = mean(u_vir_cam);
ng = mean(n_vir_cam);

mu20 = 0;
mu02 = 0;
for i = 1:length(u_vir_cam)
    
    mu_20 = (u_vir_cam(i)-ug)^2;
    mu_02 = (n_vir_cam(i)-ng)^2;
    
    mu20 = mu20 + mu_20;
    mu02 = mu02 + mu_02;
    
end

aD = mu20+mu02;
fprintf('%.7f\n',aD)