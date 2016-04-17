require 'orocos'
require 'readline'

include Orocos

Orocos.initialize

Orocos.run "kf_orientation::Task" => "orientation" do    
    # orientation communication           
    orientation_task = TaskContext.get 'orientation'
    orientation_task.apply_conf_file("kf_orientation::Task.yml")

    kvk_imu_task = TaskContext.get 'imu_kvh_1750_mixed'
    pressure_sensor_task = TaskContext.get 'pressure_sensor_digiquartz'

    kvk_imu_task.mix_samples.connect_to orientation_task.imu_sensors_cmd
    pressure_sensor_task.pressure_bar.connect_to orientation_task.pressure_bar_cmd

    orientation_task.configure
    orientation_task.start

    Readline.readline("Press ENTER to exit\n")
end
