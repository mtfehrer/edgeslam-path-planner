#Check if edgeslam image exists
if ! docker image inspect edgeslam > /dev/null 2>&1; then
    echo "Image 'edgeslam' not found. Building..."
    docker build -t edgeslam -f Dockerfile ./edgeslam/
else
    echo "Image 'edgeslam' already exists."
fi
#Check if planner image exists
if ! docker image inspect planner > /dev/null 2>&1; then
    echo "Image 'planner' not found. Building..."
    docker build -t planner -f Dockerfile ./planner/
else
    echo "Image 'planner' already exists."
fi