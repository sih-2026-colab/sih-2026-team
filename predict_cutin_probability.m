function p=predict_cutin_probability(model,track,ego,referenceY)
s=track.State; covariance=track.StateCovariance;
sigma=sqrt(max(0,covariance(1,1)+covariance(3,3)));
X=cutin_model_features(s(1)-ego.x,referenceY-s(3),s(4),s(2)-ego.vx,sigma);
z=[1 (X-model.mean)./model.scale]*model.weights;
p=1/(1+exp(-max(-40,min(40,z))));
if abs(referenceY-s(3))<=1.9, p=0; end
end
