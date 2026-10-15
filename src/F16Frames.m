function f = F16Frames()
%F16FRAMES Skaler bileşeni başta olan Hamilton kuaterniyonları.
% Dönüşüm, gövde vektörlerini seçilen Dünya eksen takımına taşır.
% Enlem/boylam radyan, irtifa ve ECEF konumu metre cinsindendir.
f.geodeticToECEF = @geodeticToECEF;
f.ecefToGeodetic = @ecefToGeodetic;
f.nedToECEF = @nedToECEF;
f.quatToDCM = @quatToDCM;
f.quatMultiply = @quatMultiply;
f.dcmToQuat = @dcmToQuat;
f.eulerToQuat = @eulerToQuat;
end

function r = geodeticToECEF(lat,lon,h,p)
% Jeodezik enlemden asal düşey eğrilik yarıçapı N ile ECEF konumu hesapla.
e2 = p.flattening*(2-p.flattening);
N = p.aEarth/sqrt(1-e2*sin(lat)^2);
r = [(N+h)*cos(lat)*cos(lon);
     (N+h)*cos(lat)*sin(lon);
     (N*(1-e2)+h)*sin(lat)];
end

function [lat,lon,h] = ecefToGeodetic(r,p)
% Enlemi yinelemeli çöz; kutupta yatay yarıçap sıfıra yaklaştığı için
% ayrı dal kullanılır. Yineleme sınırı olağan uçuş irtifaları içindir.
e2 = p.flattening*(2-p.flattening);
lon = atan2(r(2),r(1));
rho = hypot(r(1),r(2));
if rho < 1e-9
    lat = sign(r(3))*pi/2;
    h = abs(r(3))-p.aEarth*(1-p.flattening);
    return
end
lat = atan2(r(3),rho*(1-e2));
for k = 1:8
    N = p.aEarth/sqrt(1-e2*sin(lat)^2);
    h = rho/cos(lat)-N;
    next = atan2(r(3),rho*(1-e2*N/(N+h)));
    if abs(next-lat) < 1e-13
        lat = next;
        break
    end
    lat = next;
end
N = p.aEarth/sqrt(1-e2*sin(lat)^2);
h = rho/cos(lat)-N;
end

function C = nedToECEF(lat,lon)
% Sütunlar sırasıyla yerel kuzey, doğu, aşağı birim vektörleridir (ECEF).
sl = sin(lat); cl = cos(lat); so = sin(lon); co = cos(lon);
C = [-sl*co, -so, -cl*co;
     -sl*so,  co, -cl*so;
      cl,      0,  -sl];
end

function C = quatToDCM(q)
% Etkin dönüş: v_Dünya = C*v_gövde; q=[skaler;x;y;z].
q = q/norm(q);
s=q(1); x=q(2); y=q(3); z=q(4);
C = [1-2*(y*y+z*z), 2*(x*y-s*z),   2*(x*z+s*y);
     2*(x*y+s*z),   1-2*(x*x+z*z), 2*(y*z-s*x);
     2*(x*z-s*y),   2*(y*z+s*x),   1-2*(x*x+y*y)];
end

function q = quatMultiply(a,b)
% Hamilton çarpımı: a⊗b. Sıra değişirse dönüş yönü de değişir.
q = [a(1)*b(1)-dot(a(2:4),b(2:4));
     a(1)*b(2:4)+b(1)*a(2:4)+cross(a(2:4),b(2:4))];
end

function q = eulerToQuat(phi,theta,psi)
% NED'e göre ZYX sırası: baş(psi), dikilme(theta), yatış(phi).
qx = [cos(phi/2);sin(phi/2);0;0];
qy = [cos(theta/2);0;sin(theta/2);0];
qz = [cos(psi/2);0;0;sin(psi/2)];
q = quatMultiply(quatMultiply(qz,qy),qx);
end

function q = dcmToQuat(C)
% Ortogonal dönüşüm matrisini skaler-önce kuaterniyona çevir.
% Yarım tura yakın dönüşlerde duyarlılığı korumak için en büyük köşegeni seç.
traceC = trace(C);
if traceC > 0
    s = 2*sqrt(1+traceC);
    q = [s/4;(C(3,2)-C(2,3))/s;(C(1,3)-C(3,1))/s;(C(2,1)-C(1,2))/s];
elseif C(1,1) > C(2,2) && C(1,1) > C(3,3)
    s = 2*sqrt(1+C(1,1)-C(2,2)-C(3,3));
    q = [(C(3,2)-C(2,3))/s;s/4;(C(1,2)+C(2,1))/s;(C(1,3)+C(3,1))/s];
elseif C(2,2) > C(3,3)
    s = 2*sqrt(1+C(2,2)-C(1,1)-C(3,3));
    q = [(C(1,3)-C(3,1))/s;(C(1,2)+C(2,1))/s;s/4;(C(2,3)+C(3,2))/s];
else
    s = 2*sqrt(1+C(3,3)-C(1,1)-C(2,2));
    q = [(C(2,1)-C(1,2))/s;(C(1,3)+C(3,1))/s;(C(2,3)+C(3,2))/s;s/4];
end
q = q/norm(q);
end
