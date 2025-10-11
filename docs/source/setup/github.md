# Pull the Latest Changes from GitHub

Change to the correct git-enabled directory:

```bash
cd repos/common_platform
```

Stash any changes you have made:

```bash
git stash
```

Update the Git remote for the public repo so you don't need credentials. You should only need to do this step once.

```bash
git remote set-url origin "https://github.com/RoseCityRobotics/common_platform.git"
```

Pull the changes

```bash
git pull origin main
```

Re-apply your stashed changes

```bash
git stash apply
```
