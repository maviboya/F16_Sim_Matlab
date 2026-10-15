function paths=plot_stevens_validation(results)
%PLOT_STEVENS_VALIDATION Stevens ve üç üretim modeli; tüm çizim MATLAB.
% Dönüşte 3-DOF yoktur. Kayıtlı sonuçlardan: setup_f16; plot_stevens_validation
out=fullfile(fileparts(mfilename('fullpath')),'outputs');
if nargin<1,s=load(fullfile(out,'validation_results.mat'),'results');results=s.results;end
colors=[.05 .4 .7;.85 .33 .1;.1 .6 .4];styles={'-','-','--'};markers={'o','s','+'};
labels={'Düz Dünya','Dönen Dünya','3-DOF'};
paths=cell(3,1);
% Hız trimi: doğrudan değerler, kitap farkları ve CG etkisi.
r=results.trim;
[fig,tl]=canvasFigure([1500 1120],3,3,'Düz uçuş trimi: üç model / Stevens');
cleanup=onCleanup(@()close(fig));
units={'Gaz oranı','Hücum açısı (°)','Elevatör (°)'};
for j=1:3
    nexttile(tl,j);hold on
    plot(r.speedFps(2:end),r.reference(2:end,j),'kx','LineWidth',1.8,'DisplayName','Stevens');
    for m=1:3
        ok=r.valid(:,m);
        plot(r.speedFps(ok),r.values(ok,j,m),styles{m},'Color',colors(m,:), ...
            'LineWidth',1.5,'Marker',markers{m},'MarkerSize',4,'DisplayName',labels{m});
    end
    title(units{j});xlabel('Hız (ft/s)');ylabel(units{j});if j==1,legend('Location','northeast');end
    nexttile(tl,3+j);hold on;yline(0,':','HandleVisibility','off');
    for m=1:3
        ok=r.valid(:,m);
        plot(r.speedFps(ok),r.error(ok,j,m),styles{m},'Color',colors(m,:), ...
            'LineWidth',1.5,'Marker',markers{m},'MarkerSize',4);
    end
    title(['Model − Stevens: ' units{j}]);xlabel('Hız (ft/s)');ylabel('Fark');
    nexttile(tl,6+j);hold on;yline(0,':','HandleVisibility','off');
    [cg,order]=sort(r.cgFraction);
    for m=1:3
        plot(100*cg,r.cgError(order,j,m),styles{m},'Color',colors(m,:), ...
            'LineWidth',1.5,'Marker',markers{m},'MarkerSize',6);
    end
    title(['502 ft/s: ' units{j} ' farkı']);xlabel('Ağırlık merkezi (% kord)');ylabel('Model − Stevens');
end
paths{1}=fullfile(out,'trim.png');finish(fig,paths{1}, ...
    'Tablo 3.6-2/3; deniz seviyesi. 130 ft/s: geçerli trim yok, çizilmedi. Dönen Dünya: 41°K, 29°D, başlangıç izi kuzey.');
clear cleanup
% Tam dinamik modlar: her seri aynı kaynak satırlarıyla eşleştirilir.
r=results.modes;
[fig,tl]=canvasFigure([1660 1000],2,4,'Dinamik modlar: üç model / Stevens Tablo 4.2-3');
cleanup=onCleanup(@()close(fig));
names={'Dutch roll periyodu (s)','Dutch roll sönümü','Spiral zaman sabiti (s)', ...
    'Roll zaman sabiti (s)','Kısa periyot (s)','Kısa periyot sönümü','Fugoid periyodu (s)','Fugoid sönümü'};
condition={'900 | 50 bin','600 | 50 bin','900 | 0','367 | 0'};
for j=1:8
    nexttile(tl,j);
    count=3;if j<=4,count=2;end
    values=[r.reference(:,j),reshape(r.values(:,j,1:count),4,count)];
    h=bar(values,'grouped');h(1).FaceColor=[.18 .2 .22];
    for m=1:count,h(m+1).FaceColor=colors(m,:);end
    title(names{j});ylabel(names{j});xticks(1:4);xticklabels(condition);xtickangle(20);
    xlabel('Hız (ft/s) | irtifa (ft)');
    if j==1,legend([{'Stevens'},labels(1:2)],'Location','northoutside','Orientation','horizontal');end
    if j==5,legend([{'Stevens'},labels],'Location','northoutside','Orientation','horizontal');end
end
paths{2}=fullfile(out,'modes.png');finish(fig,paths{2}, ...
    {'600 ft/s / 50.000 ft: üç modelde geçerli trim yok; model sütunları boş. 3-DOF yalnız boylamsal modlarda.', ...
    'Motor ve irtifa dinamiği dahil. Dönen Dünya: konumda anlık lineerleştirme. 900 ft/s / 50.000 ft: Mach ≈ 0,93, kaynak zarfı dışında.'});
clear cleanup
% Koordineli dönüş: yalnız iki 6-DOF modeli, kaynağın gerçek noktaları.
r=results.turn;
[fig,tl]=canvasFigure([1550 1320],3,3,'Koordineli dönüş: düz ve dönen Dünya / Stevens');
cleanup=onCleanup(@()close(fig));
nexttile(tl,1);hold on
plot(r.source(:,3),r.source(:,2),'kx','MarkerSize',7,'LineWidth',1.5,'DisplayName','Stevens');
for m=1:2
    h=r.runs{m}.history;
    plot(h.eastFt,h.northFt,'Color',colors(m,:),'LineWidth',1.5,'DisplayName',labels{m});
end
axis equal;xlabel('Doğu (ft)');ylabel('Kuzey (ft)');title('Yer izi');legend('Location','best');
nexttile(tl,2);hold on
for m=1:2
    plot(r.source(:,1),r.sampleErrorFt(:,m),['-' markers{m}], ...
        'Color',colors(m,:),'LineWidth',1.5,'DisplayName',labels{m});
end
xlabel('Zaman (s)');ylabel('Stevens noktasına uzaklık (ft)');title('Aynı zamanlardaki konum hatası');xlim([0 180]);
nexttile(tl,3);hold on;yline(0,'k:','DisplayName','Stevens: yatay dönüş');
for m=1:2
    h=r.runs{m}.history;
    plot(h.timeS,h.altitudeFt,'Color',colors(m,:),'LineWidth',1.5,'DisplayName',labels{m});
end
xlabel('Zaman (s)');ylabel('İrtifa (ft)');title('Sabit kumandalarla serbest koşu');xlim([0 180]);
indices=[2 15 14];names={'Dönüş trimi: hücum açısı (°)','Dönüş trimi: elevatör (°)','Dönüş trimi: gaz oranı'};
for j=1:3
    nexttile(tl,3+j);k=indices(j);values=[r.referenceTrim(k),r.trimValues(k,:)];
    if k==2,values=rad2deg(values);end
    bh=bar(values,'FaceColor','flat');bh.CData=[.18 .2 .22;colors(1:2,:)];
    xticks(1:3);xticklabels({'Stevens','Düz Dünya','Dönen Dünya'});title(names{j});
    span=max(abs(values));ylim([min(0,min(values))-.15*span,max(0,max(values))+.15*span]);
    for k=1:3
        align='bottom';if values(k)<0,align='top';end
        text(k,values(k),sprintf('%.5g',values(k)),'VerticalAlignment',align,'HorizontalAlignment','center');
    end
end
% Ayrılma tanısı: üretim modellerinin karşılaştırmasına üçüncü uçak eklemez.
audit=results.rotatingTrajectoryCheck;
nexttile(tl,7);hold on
plot(audit.timeS,audit.altitudeECEFM,'Color',colors(2,:),'LineWidth',1.8,'DisplayName','ECEF üretim koşusu');
plot(audit.linearTimeS,audit.linearAltitudeM,'k--','LineWidth',1.3,'DisplayName','Değişen denge: lineer tahmin');
xlabel('Zaman (s)');ylabel('İrtifa (m)');title('İlk ayrılmanın neden hesabı');legend('Location','southwest');
nexttile(tl,8);
semilogy(audit.timeS,max(audit.positionErrorM,eps),'Color',colors(2,:),'LineWidth',1.5);
xlabel('Zaman (s)');ylabel('Konum farkı (m)');title('Bağımsız NED − üretim ECEF');ylim([1e-10 1e-4]);
nexttile(tl,9);hold on
plot(audit.linearTimeS,audit.coriolisForcingAltitudeM,'-','LineWidth',1.5,'DisplayName','Coriolis değişimi');
plot(audit.linearTimeS,audit.translationForcingAltitudeM-audit.coriolisForcingAltitudeM,':', ...
    'LineWidth',1.5,'DisplayName','Diğer öteleme artıkları');
plot(audit.linearTimeS,audit.rotationForcingAltitudeM,'--','LineWidth',1.5,'DisplayName','Dönme denge artığı');
xlabel('Zaman (s)');ylabel('Lineer irtifa katkısı (m)');title('Sapmayı başlatan terimler');legend('Location','best');
note=sprintf('502 ft/s, CG=0,35c, 0,3 rad/s. Düz Dünya: %.2f s (%s). Dönen Dünya: %.2f s (%s).', ...
    r.runs{1}.endTimeS,r.runs{1}.stopReason,r.runs{2}.endTimeS,r.runs{2}.stopReason);
paths{3}=fullfile(out,'turn.png');finish(fig,paths{3},{note, ...
    'Kontrolcü yok; kumandalar sabit. Alt sıra: 12 s denklem/ayrılma tanısı; 180 s kaynak doğrulaması değildir.'});
clear cleanup
fprintf('MATLAB grafikleri oluşturuldu: trim.png, modes.png, turn.png\n');
end

function [fig,layout]=canvasFigure(sizePx,rows,columns,heading)
fig=figure('Visible','off','Color','w','Position',[50 50 sizePx]);
if isprop(fig,'Theme'),fig.Theme='light';end
layout=tiledlayout(fig,rows,columns,'TileSpacing','compact','Padding','compact');
layout.OuterPosition=[.025 .105 .95 .84];
title(layout,heading,'FontName','Arial','FontSize',18,'FontWeight','bold');
end

function finish(fig,path,note)
set(findall(fig,'Type','axes'),'FontName','Arial','FontSize',10,'Box','off', ...
    'Color','w','XColor',[.15 .18 .2],'YColor',[.15 .18 .2], ...
    'XGrid','on','YGrid','on','GridAlpha',.13,'TickLabelInterpreter','none');
set(findall(fig,'Type','text'),'Color',[.15 .18 .2],'Interpreter','none');
set(findall(fig,'Type','legend'),'Color','w','TextColor',[.15 .18 .2]);
annotation(fig,'textbox',[.025 .008 .95 .07],'String',note,'EdgeColor','none', ...
    'FontName','Arial','FontSize',10,'Interpreter','none','Color',[.35 .4 .45]);
exportgraphics(fig,path,'Resolution',160);
end
