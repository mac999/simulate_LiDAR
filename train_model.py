# title: LiDAR model training
# author: taewook kang
# email: laputa99999@gmail.com
# version: draft. testing. 
#  0.1. simple model.
# date: 
#  2024.10.25. draft version. for testing. simple model
# 
# 
import pandas as pd
import torch
import torch.nn as nn
import torch.optim as optim
from sklearn.preprocessing import MinMaxScaler
from sklearn.model_selection import train_test_split
from torch.utils.data import DataLoader, TensorDataset

dataset = pd.read_csv('data_log.csv')

import numpy as np
data = np.array(dataset)

date = data[:, 0]  # Date
time = data[:, 1]  # Time
X = data[:, 2:5]  # Points (X, Y, Z)
y = data[:, 5]    # Diameter (label)

scaler = MinMaxScaler()
X_normalized = scaler.fit_transform(X)
y = y.astype(np.float64)

X_train, X_test, y_train, y_test = train_test_split(X_normalized, y, test_size=0.2, random_state=42)

X_train_tensor = torch.tensor(X_train, dtype=torch.float32)
y_train_tensor = torch.tensor(y_train, dtype=torch.float32).view(-1, 1)
X_test_tensor = torch.tensor(X_test, dtype=torch.float32)
y_test_tensor = torch.tensor(y_test, dtype=torch.float32).view(-1, 1)

train_dataset = TensorDataset(X_train_tensor, y_train_tensor)
test_dataset = TensorDataset(X_test_tensor, y_test_tensor)
train_dataloader = DataLoader(train_dataset, batch_size=10, shuffle=True)
test_dataloader = DataLoader(test_dataset, batch_size=10, shuffle=False)

class MLP(nn.Module):
    def __init__(self):
        super(MLP, self).__init__()
        self.fc1 = nn.Linear(3, 64)
        self.fc2 = nn.Linear(64, 32)
        self.fc3 = nn.Linear(32, 1)
    
    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        x = self.fc3(x)
        return x

from sklearn.model_selection import train_test_split
dataset = TensorDataset(X_train_tensor, y_train_tensor)
dataloader = DataLoader(dataset, batch_size=10, shuffle=True)

def train_model(model, dataloader, criterion, optimizer, epochs=100):
    for epoch in range(epochs):
        for inputs, labels in dataloader:
            optimizer.zero_grad()
            outputs = model(inputs)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()
        print(f'Epoch {epoch+1}/{epochs}, Loss: {loss.item()}')

model = MLP()
criterion = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=0.001)

train_model(model, dataloader, criterion, optimizer, epochs=100)

torch.save(model.state_dict(), 'mlp_model.pth')

new_point = np.array([[2.669634891, 9.617636291, 7.627092211]])
new_point_normalized = scaler.transform(new_point)
new_point_tensor = torch.tensor(new_point_normalized, dtype=torch.float32)
predicted_diameter = model(new_point_tensor).item()
print(f'Predicted diameter: {predicted_diameter}')

# RMSE using the test set
model.eval()
with torch.no_grad():
    y_pred = model(X_test_tensor)
    rmse = torch.sqrt(criterion(y_pred, y_test_tensor))
    print(f'RMSE on test set: {rmse.item()}')

# plot the predicted vs actual values
import matplotlib.pyplot as plt
plt.scatter(y_test, y_pred)
plt.xlabel('Actual Diameter')
plt.ylabel('Predicted Diameter')
plt.title('Actual vs Predicted Diameter')
plt.show()
