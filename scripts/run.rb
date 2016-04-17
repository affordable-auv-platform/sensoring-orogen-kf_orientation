require 'orocos'
require 'readline'

include Orocos

Orocos.initialize

# depends on middleware_communication to run

Orocos.run "kf_orientation::Task" => "orientation",
           "middleware_communication::Task" => "middleware_comm" do

    # middleware communication
    middleware_comm_task = TaskContext.get 'middleware_comm'
    
    # orientation communication           
    orientation_task = TaskContext.get 'orientation'
    orientation_task.apply_conf_file("kf_orientation::Task.yml")

    middleware_comm_task.imu_sensors_samples.connect_to orientation_task.imu_sensors_cmd
    middleware_comm_task.pressure_bar_samples.connect_to orientation_task.pressure_bar_cmd

    # configure task
    middleware_comm_task.configure    
    orientation_task.configure

    # start task
    middleware_comm_task.start
    orientation_task.start

    Readline.readline("Press ENTER to exit\n")
end
