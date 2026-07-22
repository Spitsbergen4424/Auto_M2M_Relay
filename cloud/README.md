# Training on Yandex Cloud (or any Ubuntu VM)

CPU VM is the right choice here: the bottleneck is Unity physics (40 agents in
one process), not the tiny 256×2+LSTM network, so a GPU would sit mostly idle.
Aim for **8–16 vCPU, ~16–32 GB RAM, Ubuntu 22.04**.

## 0. Confirm access (resolves "not sure")
```bash
curl -sSL https://storage.yandexcloud.net/yandexcloud-yc/install.sh | bash
exec -l $SHELL                 # reload PATH
yc init                        # OAuth login; prints your cloud/folder/billing
```
If `yc init` completes and lists a folder, access is real. If it errors on
billing/cloud, that must be fixed in the web console first.

## 1. Create the VM (fill in from `yc config list`)
```bash
yc compute instance create \
  --name gfsx-train \
  --zone ru-central1-a \
  --cores 8 --memory 16 \
  --create-boot-disk image-family=ubuntu-2204-lts,size=40 \
  --ssh-key ~/.ssh/id_ed25519.pub
```
Note the public IP it prints.

## 2. Provision it
```bash
scp cloud/setup_vm.sh yc-user@<IP>:~
ssh yc-user@<IP> "bash setup_vm.sh"      # ~5–10 min
```

## 3. Build the Linux env (on the workstation, when Unity is free)
Unity Editor → **Tools → URFU → Build Linux Cloud Training**
(needs the *Linux Dedicated Server Build Support* module in Unity Hub; it also
copies the grpc plugin into `x86_64/`, fixing the earlier startup timeout).
Output: `CloudBuild_Linux/`.

## 4. Upload build + config and launch
```bash
scp -r CloudBuild_Linux/* config.yaml cloud/run_training.sh yc-user@<IP>:~/train/
ssh yc-user@<IP>
cd train && bash run_training.sh gfsx_cloud_v1 ./GFSX_Simulator.x86_64 ./config.yaml
```

## 5. Monitor and retrieve
```bash
# live console:
tmux attach -t train_gfsx_cloud_v1        # detach: Ctrl-b then d
# pull results back to the workstation for local TensorBoard:
scp -r yc-user@<IP>:~/train/results ./
```

## Cost hygiene
Training runs for hours. **Stop the VM when done** (`yc compute instance stop
gfsx-train`) — a running instance bills continuously.
