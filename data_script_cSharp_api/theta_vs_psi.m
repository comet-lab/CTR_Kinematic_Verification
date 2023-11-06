subplot(2,1,1)
plot(range, [psi_deg(:,1), theta_deg(:,1)]);
grid on;
legend("Psi", "Theta");

subplot(2,1,2)
plot(range, [psi_deg(:,2), theta_deg(:,2)]);
grid on;
legend("Psi", "Theta");