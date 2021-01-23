url="https://github.com/Team-ASTRAS/Prudentia.git"
projectName="Prudentia"
branch="develop"
echo "[script]: Updating local repository from $url"

if [ ! -d $projectName ]; then #Directory doesn't exist - create via git clone
	$echo "[script]: Local repo not found - cloning"
	git clone $url
fi

cd Prudentia
echo "[script]: Switching to $branch and pulling..."
git checkout $branch
git pull

echo "[script]: Done! This window will close in 30 seconds.."
sleep 30