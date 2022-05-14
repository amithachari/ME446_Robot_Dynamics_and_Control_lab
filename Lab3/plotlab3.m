close all
% 
% figure(1)
% plot(ScopeData1(:,1), ScopeData1(:,2))
% hold on 
% plot(ScopeData1(:,1), ScopeData1(:,3))
% hold on
% plot(ScopeData1(:,1), ScopeData1(:,4))
% title('Error Plot without Mass Feedforward + PD control');
% legend('theta1 error', 'theta2 error', 'theta3 error', 'location', 'best')
% xlabel('time');
% ylabel('error');

figure(1)
plot(ScopeData1(:,1), ScopeData1(:,2))
hold on 
plot(ScopeData1(:,1), ScopeData1(:,3))
hold on
plot(ScopeData1(:,1), ScopeData1(:,4))
title('Error Plot without Mass Inverse dynamics to Feedforward control');
legend('theta1 error', 'theta2 error', 'theta3 error', 'location', 'best')
xlabel('time');
ylabel('error');


% figure(2)
% plot(ScopeData2(:,1), ScopeData2(:,2))
% hold on 
% plot(ScopeData2(:,1), ScopeData2(:,3))
% hold on
% plot(ScopeData2(:,1), ScopeData2(:,4))
% hold on 
% plot(ScopeData2(:,1), ScopeData2(:,5))
% title('Step Response without Mass Feedforward + PD control');
% legend('theta1', 'theta2', 'theta3', 'location', 'best')
% xlabel('time');
% ylabel('response');

figure(2)
plot(ScopeData2(:,1), ScopeData2(:,2))
hold on 
plot(ScopeData2(:,1), ScopeData2(:,3))
hold on
plot(ScopeData2(:,1), ScopeData2(:,4))
hold on 
plot(ScopeData2(:,1), ScopeData2(:,5))
title('Step Response without Mass Inverse dynamics to Feedforward control');
legend('theta1', 'theta2', 'theta3', 'location', 'best')
xlabel('time');
ylabel('response');