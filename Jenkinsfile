pipeline {
    agent any
    stages {
        stage('SCM') {
            steps {
                script {
                    properties([pipelineTriggers([pollSCM('* * * * *')])])
                }
                git branch: 'main', url: 'https://github.com/rg-masterclass/ros2_ci.git'
            }
        }
        stage('BUILD'){
            steps{
                sh 'cd ~/ros2_ws/src'
                sh '''
                    #!/bin/bash
                    if [ ! -d "ros2_ci" ]; then
                        git clone https://github.com/rg-masterclass/ros1_ci.git
                    else 
                        cd ros2_ci
                        git pull origin main
                    fi
                    '''
                sh 'cd /galactic_ws/src/ros2_ci'
                sh 'docker build . -t ros2_ci'
                sh 'docker run --name ros2_ci --rm -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -it ros2_ci:latest && sleep 30s'
            }
        }
        stage('TEST'){
            steps{
                sh 'docker exec ros2_ci bash -c "source /galactic_ws/install/setup.bash && colcon test --packages-select tortoisebot_waypoints --event-handler=console_direct+"'
            }
        }   
    }
}
