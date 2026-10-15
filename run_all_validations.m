function results=run_all_validations(renderPlots)
%RUN_ALL_VALIDATIONS Stevens / düz Dünya, dönen Dünya, boylamsal 3-DOF.
% Dönüşte yalnız iki 6-DOF modeli. Tek sonuç dosyası ve üç MATLAB grafiği.
if nargin<1,renderPlots=true;end
setup_f16();
out=fullfile(fileparts(mfilename('fullpath')),'validation','outputs');
if ~exist(out,'dir'),mkdir(out);end
test_f16();[results.sourceTables,results.rotatingTrajectoryCheck]=test_stevens_validation();
results.trim=compare_stevens_trim();
results.modes=compare_stevens_modes();
results.turn=validate_stevens_turn();
results.completedAt=datetime('now');
save(fullfile(out,'validation_results.mat'),'results');
if renderPlots,plot_stevens_validation(results);end
end
