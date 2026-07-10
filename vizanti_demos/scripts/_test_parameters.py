#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from std_msgs.msg import String, Bool, Int64, Float64
import json

class ParameterDemoNode(Node):
    def __init__(self):
        super().__init__('parameter_demo_node')
        
        # Publishers to indicate parameter changes
        self.string_pub = self.create_publisher(String, 'string_param_status', 10)
        self.bool_pub = self.create_publisher(Bool, 'bool_param_status', 10)
        self.int_pub = self.create_publisher(Int64, 'int_param_status', 10)
        self.float_pub = self.create_publisher(Float64, 'float_param_status', 10)
        self.array_pub = self.create_publisher(String, 'array_param_status', 10)
        
        # Flag to track if runtime demo has been executed
        self.runtime_demo_executed = False
        
        # Declare parameters with all supported types and descriptors
        self.declare_all_parameters()
        
        # Set up parameter change callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Timer to periodically publish parameter values
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        # Timer for runtime operations demo (runs once after 5 seconds)
        self.runtime_demo_timer = self.create_timer(5.0, self.demonstrate_runtime_operations)
        
        self.get_logger().info('Parameter Demo Node started!')
        self.log_all_parameters()


    def declare_all_parameters(self):
        """Declare parameters of all supported types with full descriptors"""
        
        self.declare_parameter('demo_string', 'hello_world', ParameterDescriptor(
            name='demo_string',
            type=ParameterType.PARAMETER_STRING,
            description='A demonstration string parameter',
            additional_constraints='Must be non-empty',
            read_only=False,
            dynamic_typing=False
        ))
        
        self.declare_parameter('demo_bool', True, ParameterDescriptor(
            name='demo_bool',
            type=ParameterType.PARAMETER_BOOL,
            description='A demonstration boolean parameter',
            additional_constraints='Controls feature activation',
            read_only=False
        ))
        
        self.declare_parameter('demo_int', 42, ParameterDescriptor(
            name='demo_int',
            type=ParameterType.PARAMETER_INTEGER,
            description='A demonstration integer parameter',
            additional_constraints='Range: 0-100',
            read_only=False
        ))
        
        self.declare_parameter('demo_double', 3.14159, ParameterDescriptor(
            name='demo_double',
            type=ParameterType.PARAMETER_DOUBLE,
            description='A demonstration double parameter',
            additional_constraints='Precision: 0.01',
            read_only=False
        ))
        
        self.declare_parameter('demo_string_array', ['robot1', 'robot2', 'robot3'], ParameterDescriptor(
            name='demo_string_array',
            type=ParameterType.PARAMETER_STRING_ARRAY,
            description='A demonstration string array parameter',
            additional_constraints='List of robot names',
            read_only=False
        ))
        
        self.declare_parameter('demo_bool_array', [True, False, True], ParameterDescriptor(
            name='demo_bool_array',
            type=ParameterType.PARAMETER_BOOL_ARRAY,
            description='A demonstration boolean array parameter',
            additional_constraints='Feature flags',
            read_only=False
        ))
        
        self.declare_parameter('demo_int_array', [8080, 8081, 8082], ParameterDescriptor(
            name='demo_int_array',
            type=ParameterType.PARAMETER_INTEGER_ARRAY,
            description='A demonstration integer array parameter',
            additional_constraints='Port numbers',
            read_only=False
        ))
        
        self.declare_parameter('demo_double_array', [1.0, 2.5, 3.7], ParameterDescriptor(
            name='demo_double_array',
            type=ParameterType.PARAMETER_DOUBLE_ARRAY,
            description='A demonstration double array parameter',
            additional_constraints='Coordinate points',
            read_only=False
        ))
        
        binary_data = [ord(c) for c in 'binary_data']
        self.declare_parameter('demo_byte_array', binary_data, ParameterDescriptor(
            name='demo_byte_array',
            type=ParameterType.PARAMETER_BYTE_ARRAY,
            description='A demonstration byte array parameter',
            additional_constraints='Binary data as list of integers (0-255)',
            read_only=False
        ))
        
        # Dynamic typing parameter (can change type)
        dynamic_descriptor = ParameterDescriptor(
            description='A parameter with dynamic typing',
            additional_constraints='Can be any type',
            read_only=False,
            dynamic_typing=True
        )
        self.declare_parameter('demo_dynamic', 'initial_string', dynamic_descriptor)

    def parameter_callback(self, params):
        """Callback function for parameter changes"""
        result = SetParametersResult(successful=True)
        
        for param in params:
            self.get_logger().info(f'Parameter {param.name} changed to: {param.value}')
            
            # Custom validation logic
            if param.name == 'demo_int':
                if param.type_ == Parameter.Type.INTEGER:
                    if not (0 <= param.value <= 100):
                        self.get_logger().error(f'Invalid value for {param.name}: {param.value}. Must be 0-100.')
                        result.successful = False
                        result.reason = 'Value out of valid range (0-100)'
                        break
            
            elif param.name == 'demo_string':
                if param.type_ == Parameter.Type.STRING:
                    if len(param.value.strip()) == 0:
                        self.get_logger().error(f'Invalid value for {param.name}: cannot be empty')
                        result.successful = False
                        result.reason = 'String parameter cannot be empty'
                        break
            
            elif param.name == 'demo_double':
                if param.type_ == Parameter.Type.DOUBLE:
                    if param.value < 0:
                        self.get_logger().error(f'Invalid value for {param.name}: {param.value}. Must be positive.')
                        result.successful = False
                        result.reason = 'Double parameter must be positive'
                        break
            
            elif param.name == 'demo_byte_array':
                if param.type_ == Parameter.Type.BYTE_ARRAY:
                    # Validate that all values are in byte range (0-255)
                    for byte_val in param.value:
                        if not (0 <= byte_val <= 255):
                            self.get_logger().error(f'Invalid byte value: {byte_val}. Must be 0-255.')
                            result.successful = False
                            result.reason = 'Byte array values must be in range 0-255'
                            break
        
        return result

    def timer_callback(self):
        """Periodically publish current parameter values"""
        
        try:
            # Get and publish string parameter
            string_param = self.get_parameter('demo_string')
            msg = String()
            msg.data = f"String param: {string_param.value}"
            self.string_pub.publish(msg)
            
            # Get and publish bool parameter
            bool_param = self.get_parameter('demo_bool')
            bool_msg = Bool()
            bool_msg.data = bool_param.value
            self.bool_pub.publish(bool_msg)
            
            # Get and publish int parameter
            int_param = self.get_parameter('demo_int')
            int_msg = Int64()
            int_msg.data = int_param.value
            self.int_pub.publish(int_msg)
            
            # Get and publish double parameter
            double_param = self.get_parameter('demo_double')
            float_msg = Float64()
            float_msg.data = double_param.value
            self.float_pub.publish(float_msg)
            
            # Get and publish array parameters as JSON string
            string_array_param = self.get_parameter('demo_string_array')
            bool_array_param = self.get_parameter('demo_bool_array')
            int_array_param = self.get_parameter('demo_int_array')
            double_array_param = self.get_parameter('demo_double_array')
            byte_array_param = self.get_parameter('demo_byte_array')
            
            # Convert byte array back to string for display (only printable characters)
            byte_as_string = ''.join([chr(b) for b in byte_array_param.value if 32 <= b <= 126])
            
            arrays_info = {
                'string_array': string_array_param.value,
                'bool_array': bool_array_param.value,
                'int_array': int_array_param.value,
                'double_array': double_array_param.value,
                'byte_array_raw': byte_array_param.value,
                'byte_array_as_string': byte_as_string
            }
            
            array_msg = String()
            array_msg.data = json.dumps(arrays_info, indent=2)
            self.array_pub.publish(array_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {str(e)}')

    def log_all_parameters(self):
        """Log all current parameter values"""
        self.get_logger().info('=== Current Parameter Values ===')
        
        try:
            # Get all parameters
            param_dict = {name: param.value for name, param in self._parameters.items()}
            
            for name, value in param_dict.items():
                param_type = self.get_parameter(name).type_
                if name == 'demo_byte_array':
                    # Display byte array in a more readable format
                    byte_as_string = ''.join([chr(b) for b in value if 32 <= b <= 126])
                    self.get_logger().info(f'{name} ({param_type.name}): {value} (as string: "{byte_as_string}")')
                else:
                    self.get_logger().info(f'{name} ({param_type.name}): {value}')
        except Exception as e:
            self.get_logger().error(f'Error logging parameters: {str(e)}')

    def demonstrate_runtime_operations(self):
        """Demonstrate various parameter operations (runs once)"""
        
        # Prevent multiple executions
        if self.runtime_demo_executed:
            return
        
        self.runtime_demo_executed = True
        
        # Cancel the timer to prevent it from running again
        self.runtime_demo_timer.cancel()
        
        try:
            self.get_logger().info('=== Demonstrating Runtime Parameter Operations ===')
            
            # Set string parameter programmatically
            result = self.set_parameters([Parameter('demo_string', Parameter.Type.STRING, 'updated_from_code')])
            if result[0].successful:
                self.get_logger().info('Successfully updated demo_string parameter')
            
            # Set byte array parameter with new data - use proper method
            new_byte_data = [72, 101, 108, 108, 111]  # "Hello" in ASCII
            
            # Use set_parameters_atomically for better error handling
            try:
                result = self.set_parameters([Parameter('demo_byte_array', Parameter.Type.BYTE_ARRAY, new_byte_data)])
                if result[0].successful:
                    self.get_logger().info('Successfully updated demo_byte_array parameter')
                else:
                    self.get_logger().error(f'Failed to update demo_byte_array: {result[0].reason}')
            except Exception as e:
                self.get_logger().error(f'Error setting byte array parameter: {str(e)}')
            
            # Check if parameter exists
            if self.has_parameter('demo_bool'):
                self.get_logger().info('demo_bool parameter exists')
            
            # Get parameter descriptor
            try:
                descriptor = self.describe_parameter('demo_int')
                self.get_logger().info(f'demo_int descriptor type: {descriptor.type}')
                self.get_logger().info(f'demo_int descriptor description: {descriptor.description}')
            except Exception as e:
                self.get_logger().error(f'Error getting parameter descriptor: {str(e)}')
            
            # Demonstrate dynamic parameter type change
            try:
                # Change dynamic parameter from string to integer
                result = self.set_parameters([Parameter('demo_dynamic', Parameter.Type.INTEGER, 123)])
                if result[0].successful:
                    self.get_logger().info('Successfully changed demo_dynamic from string to integer')
                
                # Change it back to string
                result = self.set_parameters([Parameter('demo_dynamic', Parameter.Type.STRING, 'back_to_string')])
                if result[0].successful:
                    self.get_logger().info('Successfully changed demo_dynamic back to string')
            except Exception as e:
                self.get_logger().error(f'Error with dynamic parameter: {str(e)}')
                
        except Exception as e:
            self.get_logger().error(f'Error in runtime operations demo: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    node = ParameterDemoNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down parameter demo node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()