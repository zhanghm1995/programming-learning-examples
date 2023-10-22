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
print("y_train:", y_train.shape)
print("X_test:", X_test.shape)
print("y_test:", y_test.shape)
