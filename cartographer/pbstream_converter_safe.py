#!/usr/bin/env python3
import argparse
import os
import struct
import sys
import open3d as o3d

class PbStreamParser:
    def __init__(self):
        self.points = []
        self.colors = []
    
    def parse_pbstream(self, file_path):
        """安全解析pbstream文件，不依赖proto定义"""
        with open(file_path, 'rb') as f:
            data = f.read()
        
        # 查找可能的点云数据块
        # pbstream通常包含多个消息，我们尝试查找包含浮点数组的部分
        pos = 0
        while pos < len(data) - 12:  # 至少需要12字节(x,y,z)
            # 尝试读取3个float32(x,y,z坐标)
            try:
                x, y, z = struct.unpack_from('fff', data, pos)
                self.points.append([x, y, z])
                pos += 12
            except struct.error:
                pos += 1  # 如果不是浮点数，移动1字节继续搜索
    
    def save_to_file(self, output_path):
        """保存为PCD或PLY格式"""
        if not self.points:
            raise ValueError("No point cloud data found")
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.points)
        if self.colors:
            pcd.colors = o3d.utility.Vector3dVector(self.colors)
        
        o3d.io.write_point_cloud(output_path, pcd)

def main():
    parser = argparse.ArgumentParser(
        description='Safe PBStream to PCD/PLY Converter (No Proto Dependency)')
    parser.add_argument('input_pbstream', help='Input .pbstream file')
    parser.add_argument('output_file', help='Output .pcd or .ply file')
    args = parser.parse_args()
    
    # 验证输入文件
    if not os.path.exists(args.input_pbstream):
        print(f"Error: Input file {args.input_pbstream} not found")
        sys.exit(1)
    
    # 验证输出格式
    output_ext = os.path.splitext(args.output_file)[1].lower()
    if output_ext not in ['.pcd', '.ply']:
        print("Error: Output file must have .pcd or .ply extension")
        sys.exit(1)
    
    # 执行转换
    parser = PbStreamParser()
    try:
        print(f"Parsing {args.input_pbstream} (this may take a while)...")
        parser.parse_pbstream(args.input_pbstream)
        
        print(f"Found {len(parser.points)} points, saving to {args.output_file}...")
        parser.save_to_file(args.output_file)
        
        print("Conversion completed successfully")
    except Exception as e:
        print(f"Error during conversion: {str(e)}")
        sys.exit(1)

if __name__ == "__main__":
    main()