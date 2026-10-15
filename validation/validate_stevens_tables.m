function result = validate_stevens_tables()
%VALIDATE_STEVENS_TABLES Ek A.7/A.8'in bütün 852 tablo sayısını bağımsız sınar.
% Referans JSON, üretim MATLAB/Excel dosyasından değil yayıncıdan aktarılmıştır.
root=fileparts(mfilename('fullpath'));
r=jsondecode(fileread(fullfile(root,'references','stevens_appendix_tables.json')));
p=F16Parameters(); V=150; rho=1.2; sound=340;
alpha=-10:5:45; betapos=0:5:30; beta=-30:10:30; elevator=-24:12:24;
names={}; errors=[]; count=[];
for key={'CX','CM','CL','CN','DLDA','DLDR','DNDA','DNDR','CZ','DAMP'}
    name=key{1}; expected=r.(name);
    if strcmp(name,'CZ'), expected=expected(:)'; end
    actual=zeros(size(expected));
    for j=1:size(expected,1)
        for k=1:numel(alpha)
            b=0; de=0; da=0; dr=0; w=zeros(3,1);
            switch name
                case {'CX','CM'}, de=elevator(j);
                case {'CL','CN'}, b=betapos(j);
                case {'DLDA','DNDA'}, b=beta(j); da=20;
                case {'DLDR','DNDR'}, b=beta(j); dr=30;
                case 'DAMP'
                    if ismember(j,[1 4 7]), w(2)=2*V/p.chord;
                    elseif ismember(j,[2 5 8]), w(3)=2*V/p.span;
                    else, w(1)=2*V/p.span; end
            end
            c=coeff(alpha(k),b,de,da,dr,w);
            c0=coeff(alpha(k),b,0,0,0,zeros(3,1));
            switch name
                case 'CX', actual(j,k)=c(1);
                case 'CM', actual(j,k)=c(5);
                case 'CL', actual(j,k)=c(4);
                case 'CN', actual(j,k)=c(6);
                case {'DLDA','DLDR'}, actual(j,k)=c(4)-c0(4);
                case {'DNDA','DNDR'}, actual(j,k)=c(6)-c0(6);
                case 'CZ', actual(j,k)=c(3);
                case 'DAMP'
                    component=[1 2 2 3 4 4 5 6 6];
                    actual(j,k)=c(component(j))-c0(component(j));
            end
        end
    end
    names{end+1,1}=name; errors(end+1,1)=max(abs(actual-expected),[],'all');
    count(end+1,1)=numel(expected);
end
for k=1:3
    key={'idle','mil','maximum'}; expected=r.(key{k}); actual=zeros(6);
    for i=1:6
        for j=1:6
            F=F16ThrustRK4(.5,(k-1)*50,(j-1)*10000*.3048,(i-1)*.2,p);
            actual(i,j)=F(1)/4.4482216152605;
        end
    end
    names{end+1,1}=key{k};errors(end+1,1)=max(abs(actual-expected),[],'all');
    count(end+1,1)=numel(expected);
end
% Düğüm aralarında, her iki beta işaretinde ve aynı anda kumanda/sönüm.
offGridError=0;
for k=1:120
    a=-9.5+54*mod(k*37,121)/121;
    b=-29.5+59*mod(k*53,127)/127;
    de=-23.5+47*mod(k*17,131)/131;
    da=17*sin(k);dr=23*cos(k);w=[.1*sin(k);.2*cos(k);-.15*sin(2*k)];
    d=zeros(9,1);
    for j=1:9,d(j)=interp1(alpha,r.DAMP(j,:),a);end
    pn=w(1)*p.span/(2*V);qn=w(2)*p.chord/(2*V);rn=w(3)*p.span/(2*V);
    expected=[interp2(alpha,elevator,r.CX,a,de)+qn*d(1); ...
        -.02*b+.021*da/20+.086*dr/30+rn*d(2)+pn*d(3); ...
        interp1(alpha,r.CZ,a)*(1-deg2rad(b)^2)-.19*de/25+qn*d(4); ...
        (sign(b)*interp2(alpha,betapos,r.CL,a,abs(b)) ...
            +interp2(alpha,beta,r.DLDA,a,b)*da/20 ...
            +interp2(alpha,beta,r.DLDR,a,b)*dr/30+rn*d(5)+pn*d(6)); ...
        interp2(alpha,elevator,r.CM,a,de)+qn*d(7); ...
        (sign(b)*interp2(alpha,betapos,r.CN,a,abs(b)) ...
            +interp2(alpha,beta,r.DNDA,a,b)*da/20 ...
            +interp2(alpha,beta,r.DNDR,a,b)*dr/30+rn*d(8)+pn*d(9))];
    offGridError=max(offGridError,max(abs(coeff(a,b,de,da,dr,w)-expected)));
end
names{end+1,1}='offGridCoefficients';count(end+1,1)=120*6;errors(end+1,1)=offGridError;
result=table(names,count,errors,'VariableNames',{'table','nodeCount','maxAbsError'});
assert(all(errors<1e-9),'Stevens source table mismatch.');
disp(result);
    function c=coeff(a,b,de,da,dr,w)
        v=V*[cosd(a)*cosd(b);sind(b);sind(a)*cosd(b)];
        [F,M,~,~,~,q]=AeroModel(v,w,rho,p.Sref,p.span,p.chord, ...
            de,da,dr,sound,zeros(3,1),zeros(3,1));
        c=[F/(q*p.Sref); M./(q*p.Sref*[p.span;p.chord;p.span])];
    end
end
