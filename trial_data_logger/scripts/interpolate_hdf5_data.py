import h5py
import pandas as pd
import numpy as np
import os
import glob


def interpolate_hdf5_data(file_path, common_time_base='5ms'):
    with h5py.File(file_path, 'r') as file:
        datasets = list(file['TrialData'].keys())
        datasets_data = {}

        for dataset in datasets:
            data = file['TrialData'][dataset][()]
            timestamps = data[:, 0]
            data_values = data[:, 1:]
            df = pd.DataFrame(data_values, index=pd.to_datetime(timestamps, unit='s'))
            datasets_data[dataset] = df

    interpolated_data = {}
    for dataset_name, df in datasets_data.items():
        resampled_df = df.resample(common_time_base).mean()
        interpolated_df = resampled_df.interpolate(method='time')
        interpolated_data[dataset_name] = interpolated_df

    output_file = os.path.splitext(file_path)[0] + '_interpolated.h5'
    with h5py.File(output_file, 'w') as file:
        for dataset_name, df in interpolated_data.items():
            data = df.to_numpy()
            timestamps = df.index.astype(np.int64) // 10 ** 9
            data_with_time = np.column_stack((timestamps, data))
            file.create_dataset('TrialData/' + dataset_name, data=data_with_time)

    print(f'Interpolated data saved to {output_file}')


def main():
    subject_num = input("Enter the subject number: ")
    current_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.join(current_dir, 'DATA')
    subject_dir = os.path.join(data_dir, f'Subject{subject_num}')
    calibration_dir = os.path.join(subject_dir, 'Calibration')
    fitting_dir = os.path.join(subject_dir, 'Fitting')
    methods = ['MIntNet', 'LinearFitting', 'CircleFitting']

    for method in methods:
        method_cal_dir = os.path.join(calibration_dir, method)
        method_fit_dir = os.path.join(fitting_dir, method)

        if os.path.exists(method_cal_dir):
            hdf5_files = glob.glob(os.path.join(method_cal_dir, '*.h5'))
            for file_path in hdf5_files:
                print(f"Interpolating file in Calibration: {file_path}")
                interpolate_hdf5_data(file_path)

        if os.path.exists(method_fit_dir):
            hdf5_files = glob.glob(os.path.join(method_fit_dir, '*.h5'))
            for file_path in hdf5_files:
                print(f"Interpolating file in Fitting: {file_path}")
                interpolate_hdf5_data(file_path)


if __name__ == '__main__':
    main()
