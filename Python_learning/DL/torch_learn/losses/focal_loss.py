import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset
from sklearn.metrics import accuracy_score, precision_score, recall_score, f1_score


class FocalLoss(nn.Module):
    def __init__(self, alpha=1, gamma=2, reduction='mean'):
        super(FocalLoss, self).__init__()
        self.alpha = alpha  # balancing parameter
        self.gamma = gamma  # focusing parameter
        self.reduction = reduction

    def forward(self, inputs, targets):
        # Compute binary cross entropy loss
        bce_loss = F.binary_cross_entropy_with_logits(inputs, targets, reduction='none')

        # Compute the modulating factor
        modulating_factor = torch.exp(-self.alpha * targets * (inputs.sigmoid().log()))
        
        # Compute the focal loss
        focal_loss = (1 - inputs.sigmoid()).pow(self.gamma) * bce_loss * modulating_factor

        if self.reduction == 'mean':
            focal_loss = focal_loss.mean()
        elif self.reduction == 'sum':
            focal_loss = focal_loss.sum()

        return focal_loss


# Define the binary classification model
class BinaryClassificationModel(nn.Module):
    def __init__(self):
        super(BinaryClassificationModel, self).__init__()
        self.fc1 = nn.Linear(20, 10)
        self.fc2 = nn.Linear(10, 1)

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = self.fc2(x)
        return x


import torch
import numpy as np
from sklearn.model_selection import train_test_split
from torch.utils.data import DataLoader, TensorDataset

# Generate synthetic data
# Assume you have two classes (0 and 1)

# Class 0 data (randomly generated)
class_0_data = np.random.normal(loc=5, scale=2, size=(100, 20))

# Class 1 data (randomly generated)
class_1_data = np.random.normal(loc=10, scale=2, size=(100, 20))

# Labels for the data
class_0_labels = np.zeros((100, 1))  # Class 0 is labeled as 0
class_1_labels = np.ones((100, 1))   # Class 1 is labeled as 1

# Concatenate the data and labels for both classes
X = np.vstack((class_0_data, class_1_data))
y = np.vstack((class_0_labels, class_1_labels))

# Split the data into training and testing sets
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Convert data to PyTorch tensors
X_train = torch.tensor(X_train, dtype=torch.float32)
y_train = torch.tensor(y_train, dtype=torch.float32)
X_test = torch.tensor(X_test, dtype=torch.float32)
y_test = torch.tensor(y_test, dtype=torch.float32)

# Create DataLoader for training and testing
train_dataset = TensorDataset(X_train, y_train)
train_loader = DataLoader(train_dataset, batch_size=32, shuffle=True)

test_dataset = TensorDataset(X_test, y_test)
test_loader = DataLoader(test_dataset, batch_size=32, shuffle=False)

# Display the shapes of the prepared data
print("Shapes of prepared data:")
print("X_train:", X_train.shape)
print("y_train:", y_train, y_train.shape)
print("X_test:", X_test.shape)
print("y_test:", y_test, y_test.shape)


# Instantiate the model and FocalLoss
model = BinaryClassificationModel()
focal_loss = FocalLoss(alpha=1, gamma=2)

# Define optimizer
optimizer = optim.Adam(model.parameters(), lr=0.01)

# Training loop
num_epochs = 20
for epoch in range(num_epochs):
    model.train()
    total_loss = 0.0

    for inputs, targets in train_loader:
        # Forward pass
        outputs = model(inputs)
        loss = focal_loss(outputs.squeeze(), targets.squeeze())  # Calculate Focal Loss

        # Backward and optimize
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        total_loss += loss.item()

    # Print average loss for this epoch
    print(f"Epoch [{epoch+1}/{num_epochs}], Loss: {total_loss/len(train_loader):.4f}")

# Inference
# Assuming you have a test dataset X_test
# Replace with your actual test data
# X_test should be a PyTorch tensor or numpy array

model.eval()
with torch.no_grad():
    # Forward pass on the test data
    predictions = model(X_test)
    predicted_labels = (predictions.sigmoid() > 0.5).float()

# Convert predicted labels to numpy array
predicted_labels_np = predicted_labels.cpu().numpy()
print("Predicted Labels:", predicted_labels_np)

# Convert predicted labels to numpy array
predicted_labels_np = predicted_labels.cpu().numpy()

# Calculate evaluation metrics
accuracy = accuracy_score(y_test, predicted_labels_np)
precision = precision_score(y_test, predicted_labels_np)
recall = recall_score(y_test, predicted_labels_np)
f1 = f1_score(y_test, predicted_labels_np)

# Print evaluation metrics
print("Accuracy:", accuracy)
print("Precision:", precision)
print("Recall:", recall)
print("F1 Score:", f1)