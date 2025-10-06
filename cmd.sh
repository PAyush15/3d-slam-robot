# Remove any broken cmake remnants
sudo apt-get purge --auto-remove cmake -y
sudo apt-get update

# Add Kitware official repository (maintainers of CMake)
sudo apt-get install -y apt-transport-https ca-certificates gnupg software-properties-common curl
sudo curl -fsSL https://apt.kitware.com/keys/kitware-archive-latest.asc | sudo gpg --dearmor -o /usr/share/keyrings/kitware-archive-keyring.gpg
echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main' | sudo tee /etc/apt/sources.list.d/kitware.list
sudo apt-get update

# Install newer cmake from Kitware
sudo apt-get install -y cmake
