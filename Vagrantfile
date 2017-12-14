Vagrant.configure(2) do |config|
  config.vm.box = "debian/stretch64"

  config.vm.box_check_update = false
  config.vm.network "public_network"

  config.vm.provider :virtualbox do |vb|
    vb.customize ["modifyvm", :id, "--usb", "on"]
    vb.customize ["usbfilter", "add", "0", "--target", :id,
                  "--name", "XAP-EA-001",
                  "--vendorid", "0x10c4", "--productid", "0x8BDE"]
  end

  config.vm.synced_folder ".", "/git"

  config.vm.provision "shell", inline: <<-SHELL
    sudo apt-get update
    sudo apt-get install -y gcc make                          \
                            dpkg-dev usbutils                 \
                            linux-headers-$(uname -r)
    sudo apt-get autoremove -y

  SHELL
  
end
