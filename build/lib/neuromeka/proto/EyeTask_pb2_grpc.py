# Generated by the gRPC Python protocol compiler plugin. DO NOT EDIT!
"""Client and server classes corresponding to protobuf-defined services."""
import grpc

import EyeTask_pb2 as EyeTask__pb2


class EyeTaskStub(object):
    """Missing associated documentation comment in .proto file."""

    def __init__(self, channel):
        """Constructor.

        Args:
            channel: A grpc.Channel.
        """
        self.GetImage = channel.unary_unary(
                '/EyeTask.EyeTask/GetImage',
                request_serializer=EyeTask__pb2.ImageRequest.SerializeToString,
                response_deserializer=EyeTask__pb2.ImageResponse.FromString,
                )
        self.GetClassList = channel.unary_unary(
                '/EyeTask.EyeTask/GetClassList',
                request_serializer=EyeTask__pb2.Request.SerializeToString,
                response_deserializer=EyeTask__pb2.ClassList.FromString,
                )
        self.Detect = channel.unary_unary(
                '/EyeTask.EyeTask/Detect',
                request_serializer=EyeTask__pb2.DetectRequest.SerializeToString,
                response_deserializer=EyeTask__pb2.DetectResponse.FromString,
                )
        self.Retrieve = channel.unary_unary(
                '/EyeTask.EyeTask/Retrieve',
                request_serializer=EyeTask__pb2.RetrieveRequest.SerializeToString,
                response_deserializer=EyeTask__pb2.DetectResponse.FromString,
                )


class EyeTaskServicer(object):
    """Missing associated documentation comment in .proto file."""

    def GetImage(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def GetClassList(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def Detect(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def Retrieve(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')


def add_EyeTaskServicer_to_server(servicer, server):
    rpc_method_handlers = {
            'GetImage': grpc.unary_unary_rpc_method_handler(
                    servicer.GetImage,
                    request_deserializer=EyeTask__pb2.ImageRequest.FromString,
                    response_serializer=EyeTask__pb2.ImageResponse.SerializeToString,
            ),
            'GetClassList': grpc.unary_unary_rpc_method_handler(
                    servicer.GetClassList,
                    request_deserializer=EyeTask__pb2.Request.FromString,
                    response_serializer=EyeTask__pb2.ClassList.SerializeToString,
            ),
            'Detect': grpc.unary_unary_rpc_method_handler(
                    servicer.Detect,
                    request_deserializer=EyeTask__pb2.DetectRequest.FromString,
                    response_serializer=EyeTask__pb2.DetectResponse.SerializeToString,
            ),
            'Retrieve': grpc.unary_unary_rpc_method_handler(
                    servicer.Retrieve,
                    request_deserializer=EyeTask__pb2.RetrieveRequest.FromString,
                    response_serializer=EyeTask__pb2.DetectResponse.SerializeToString,
            ),
    }
    generic_handler = grpc.method_handlers_generic_handler(
            'EyeTask.EyeTask', rpc_method_handlers)
    server.add_generic_rpc_handlers((generic_handler,))


 # This class is part of an EXPERIMENTAL API.
class EyeTask(object):
    """Missing associated documentation comment in .proto file."""

    @staticmethod
    def GetImage(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/EyeTask.EyeTask/GetImage',
            EyeTask__pb2.ImageRequest.SerializeToString,
            EyeTask__pb2.ImageResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def GetClassList(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/EyeTask.EyeTask/GetClassList',
            EyeTask__pb2.Request.SerializeToString,
            EyeTask__pb2.ClassList.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def Detect(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/EyeTask.EyeTask/Detect',
            EyeTask__pb2.DetectRequest.SerializeToString,
            EyeTask__pb2.DetectResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def Retrieve(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/EyeTask.EyeTask/Retrieve',
            EyeTask__pb2.RetrieveRequest.SerializeToString,
            EyeTask__pb2.DetectResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)
