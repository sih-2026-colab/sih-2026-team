function metrics=train_autonex_intent_model()
% Reproducible synthetic-data logistic baseline; no real-road accuracy claim.
root=fileparts(mfilename('fullpath')); folder=fullfile(root,'models');
if ~isfolder(folder), mkdir(folder); end
[X,y]=dataset(6000,135); [V,vy]=dataset(2000,246); [T,ty]=dataset(2000,357);
model.mean=mean(X,1); model.scale=max(std(X,[],1),.01);
A=[ones(size(X,1),1) (X-model.mean)./model.scale]; w=zeros(size(A,2),1);
for iteration=1:1000
    p=sigmoid(A*w);
    gradient=A'*(p-y)/numel(y)+.002*[0;w(2:end)];
    w=w-.12*gradient;
end
model.weights=w; model.featureNames={'absLateralDistance','towardMotion', ...
    'absLateralSpeed','timeToCorridor','relativeX','relativeVx','positionSigma'};
model.trainingSource='SYNTHETIC_KINEMATIC_TRAJECTORIES_ONLY';
model.label='Initially outside 1.9m corridor, then enters within 3 seconds';
model.seeds=[135 246 357];
vp=sigmoid([ones(size(V,1),1) (V-model.mean)./model.scale]*w);
thresholds=.05:.025:.95; cost=zeros(size(thresholds));
for k=1:numel(thresholds)
    pred=vp>=thresholds(k); cost(k)=mean(3*(~pred & vy)+(pred & ~vy));
end
[~,k]=min(cost); model.threshold=thresholds(k);
tp=sigmoid([ones(size(T,1),1) (T-model.mean)./model.scale]*w);
pred=tp>=model.threshold; positives=sum(ty); negatives=sum(~ty);
[~,order]=sort(tp); ranks=zeros(size(tp)); ranks(order)=1:numel(tp);
metrics=struct('source',model.trainingSource,'trainCount',numel(y),'validationCount',numel(vy), ...
    'testCount',numel(ty),'threshold',model.threshold,'accuracy',mean(pred==ty), ...
    'precision',sum(pred & ty)/max(sum(pred),1),'recall',sum(pred & ty)/max(positives,1), ...
    'brierScore',mean((tp-ty).^2), ...
    'auc',(sum(ranks(logical(ty)))-positives*(positives+1)/2)/max(positives*negatives,1));
metrics.prototypeGate=metrics.recall>=.7 && metrics.auc>=.8 && metrics.brierScore<.25;
save(fullfile(folder,'autonex_intent_model.mat'),'model','metrics');
save(fullfile(folder,'synthetic_intent_dataset.mat'),'X','y','V','vy','T','ty');
fid=fopen(fullfile(root,'results','intent_model_metrics.json'),'w'); cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(metrics,PrettyPrint=true)); disp(metrics);
assert(metrics.prototypeGate,'Synthetic intent baseline did not meet the prototype gate.');
end
function [X,label]=dataset(n,seed)
rng(seed); dy=12*rand(n,1)-6; lateralV=5*rand(n,1)-2.5;
dx=55*rand(n,1)-10; relativeVx=10*rand(n,1)-5;
sigma=.1+.9*rand(n,1); acceleration=.6*randn(n,1);
future=dy-lateralV*(.25:.25:3)-.5*acceleration*((.25:.25:3).^2);
label=double(abs(dy)>1.9 & any(abs(future)<=1.9,2));
X=cutin_model_features(dx,dy+.3*sigma.*randn(n,1), ...
    lateralV+.2*sigma.*randn(n,1),relativeVx,sigma);
end
function p=sigmoid(z)
p=1./(1+exp(-max(-40,min(40,z))));
end
