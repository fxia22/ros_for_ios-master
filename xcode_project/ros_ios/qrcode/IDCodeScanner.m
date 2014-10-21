//
//  IDCodeScanner.m
//  SquareCam 
//
//  Created by Ivan Djeferov on 10/5/13.
//  Copyright (c) 2013 Incept Development. All rights reserved.
//


#import "IDCodeScanner.h"


@interface IDCodeScanner () <AVCaptureMetadataOutputObjectsDelegate>

// AV Foundation variables

// All the capture sessions
@property (nonatomic, strong) AVCaptureSession *captureSession;

// Every capture session can have a layer
@property (nonatomic, strong) AVCaptureVideoPreviewLayer *previewLayer;

// Used to store temporarily the metadatata objects
@property (nonatomic, strong) NSArray *currentlyRecognizedMetadataObjects;

// Callback on callback queue with error
- (void)callbackWithError:(NSError *)error;

@end


@implementation IDCodeScanner

#pragma mark - Init

- (id)init
{
    self = [super init];
    
    if (self)
    {
        // Set default values
        _capturePosition = AVCaptureDevicePositionUnspecified;
        self.callbackQueue = dispatch_get_main_queue();
    }
    
    return self;
}

#pragma mark - Setup

- (void)startCaptureSessionForCapturePosition:(AVCaptureDevicePosition)capturePosition andMetadataObjectTypes:(NSArray *)metadataObjectTypes
{
    [self startCaptureSessionForCapturePosition:capturePosition parameters:nil andMetadataObjectTypes:metadataObjectTypes];
}

- (void)startCaptureSessionForCapturePosition:(AVCaptureDevicePosition)capturePosition parameters:(NSDictionary *)parameters andMetadataObjectTypes:(NSArray *)metadataObjectTypes
{
    // Error handling
	NSError *error = nil;

    // Sanity checks
    if (capturePosition == AVCaptureDevicePositionUnspecified)
    {
        [self stopCaptureSession];
        return;
    }

    if (![metadataObjectTypes isKindOfClass:[NSArray class]])
    {
        // TODO: create error
        [self callbackWithError:nil];
        return;
    }

    // Init capture session
	self.captureSession = [[AVCaptureSession alloc] init];

    // Decide on capture session preset (important for video quality)
    NSString *sessionPreset = [parameters objectForKey:kIDCodeScannerParametersSessionPresetKey];

    if ([sessionPreset isKindOfClass:[NSString class]] && (sessionPreset.length > 0))
    {
        self.captureSession.sessionPreset = sessionPreset;
    }
    else // Unspecified session preset
    {
        if ([[UIDevice currentDevice] userInterfaceIdiom] == UIUserInterfaceIdiomPhone)
        {
            [self.captureSession setSessionPreset:AVCaptureSessionPreset640x480];
        }
        else // Presumably an iPad
        {
            [self.captureSession setSessionPreset:AVCaptureSessionPresetPhoto];
        }
    }

    // Select a video device, make an input
	AVCaptureDevice *device;

    // Enumerate the devices and select the proper one
    NSArray *devices = [AVCaptureDevice devicesWithMediaType: AVMediaTypeVideo];
    
    for (AVCaptureDevice *foundDevice in devices)
    {
        if (foundDevice.position == capturePosition)
        {
            device = foundDevice;
            _capturePosition = device.position;
            break;
        }
    }

    // No device found for desired position
    if (!device)
    {
        // TODO: create error
        [self callbackWithError:nil];
        return;
    }

    // Create a device input
	AVCaptureDeviceInput *captureDeviceInput = [AVCaptureDeviceInput deviceInputWithDevice:device error:&error];

    // Callback with an error
    if (error)
    {
        [self callbackWithError:error];
        return;
    }

    // Add device as an input for the capture session
	if ( [self.captureSession canAddInput:captureDeviceInput] )
    {
		[self.captureSession addInput:captureDeviceInput];
    }
    else
    {
        // TODO: create error
        [self callbackWithError:nil];
        return;
    }

    // Make a video data output
	AVCaptureVideoDataOutput *videoDataOutput = [[AVCaptureVideoDataOutput alloc] init];

    // we want BGRA, both CoreGraphics and OpenGL work well with 'BGRA'
	NSDictionary *rgbOutputSettings = @{ (id)kCVPixelBufferPixelFormatTypeKey : @(kCMPixelFormat_32BGRA) };
	[videoDataOutput setVideoSettings:rgbOutputSettings];
	[videoDataOutput setAlwaysDiscardsLateVideoFrames:YES]; // discard if the data output queue is blocked (as we process the still image)

    // Add video data output to capture session
    if ( [self.captureSession canAddOutput:videoDataOutput] )
    {
		[self.captureSession addOutput:videoDataOutput];
    }
    else
    {
        // TODO: create error
        [self callbackWithError:nil];
        return;
    }

    // Create a mapture metadata output to capture the codes
    AVCaptureMetadataOutput *codeMetaOutput = [[AVCaptureMetadataOutput alloc] init];

    // Set metadata objects delegate object and callback queue
    [codeMetaOutput setMetadataObjectsDelegate:self queue:self.callbackQueue];

    // Add metadata output to the current capture session
    if ( [self.captureSession canAddOutput:codeMetaOutput] )
    {
        [self.captureSession addOutput:codeMetaOutput];
    }

    // Set metadata objects to recognize
    NSMutableArray *supportedMetadataObjectTypes = [NSMutableArray array];

    // Firstly we check for supported types
    for (NSString *metadataObjectType in metadataObjectTypes)
    {
        if ([codeMetaOutput.availableMetadataObjectTypes containsObject:metadataObjectType])
        {
            [supportedMetadataObjectTypes addObject:metadataObjectType];
        }
    }

    if (!supportedMetadataObjectTypes.count)
    {
        // TODO: create error
        [self callbackWithError:nil];
        return;
    }

    // We set the supported metadata object types
    codeMetaOutput.metadataObjectTypes = self.currentlyRecognizedMetadataObjects = supportedMetadataObjectTypes;

    // Create video layer
	self.previewLayer = [[AVCaptureVideoPreviewLayer alloc] initWithSession:self.captureSession];
	[self.previewLayer setBackgroundColor:[[UIColor blackColor] CGColor]];
	[self.previewLayer setVideoGravity:AVLayerVideoGravityResizeAspectFill];

    // Start running the capture session
	[self.captureSession startRunning];

    // Callback with success
    if (self.shouldAddLayerFromCaptureSessionToView)
    {
        // We call back on the callbackQueue
        dispatch_async(self.callbackQueue, ^
        {
            self.shouldAddLayerFromCaptureSessionToView (self, capturePosition, self.captureSession, self.previewLayer);
        });
    }
}

- (void)stopCaptureSession
{
    [self.previewLayer removeFromSuperlayer];
    self.previewLayer = nil;

    [self.captureSession stopRunning];
    self.captureSession = nil;

    self.currentlyRecognizedMetadataObjects = nil;
}

- (void)callbackWithError:(NSError *)error
{
    assert([error isKindOfClass:[NSError class]]);

    id callback = ^()
    {
        if (self.didFailWithError)
        {
            self.didFailWithError(self, error);
        }
    };

    dispatch_queue_t queue = dispatch_get_main_queue();

    if (self.callbackQueue)
    {
        queue = self.callbackQueue;
    }

    dispatch_async(queue, callback);
}

#pragma mark - Starting a session for a given camera

- (void)setCapturePosition:(AVCaptureDevicePosition)capturePosition
{
    if (capturePosition != _capturePosition)
    {
        @synchronized (self)
        {
            switch (capturePosition)
            {
                case AVCaptureDevicePositionFront:
                case AVCaptureDevicePositionBack:
                {
                    [self startCaptureSessionForCapturePosition:capturePosition
                                         andMetadataObjectTypes:self.currentlyRecognizedMetadataObjects];
                    break;
                }
                case AVCaptureDevicePositionUnspecified:
                default:
                {
                    [self stopCaptureSession];
                    break;
                }
            }

            _capturePosition = capturePosition;
        }
    }
}

#pragma mark - Switch between cameras

- (BOOL)switchCamera
{
    // Check if we really have two cameras
    if (AVCaptureDevice.devices.count > 1)
    {
        AVCaptureDevicePosition desiredPosition;

        switch (self.capturePosition)
        {
            case AVCaptureDevicePositionFront:
            {
                desiredPosition = AVCaptureDevicePositionBack;
                break;
            }
            case AVCaptureDevicePositionBack:
            {
                desiredPosition = AVCaptureDevicePositionFront;
                break;
            }
            default:
            {
                // Nope
                return NO;
            }
        }

        // Enumerate all video capture devices
        NSArray *videoDevices = [AVCaptureDevice devicesWithMediaType: AVMediaTypeVideo];
        
        for (AVCaptureDevice *captureDevice in videoDevices)
        {
            if (captureDevice.position == desiredPosition)
            {
                // Start configuring capture session
                [self.captureSession beginConfiguration];
                
                // Error check
                NSError *err;
                
                // Get capture device input for capture device
                AVCaptureDeviceInput *input = [AVCaptureDeviceInput deviceInputWithDevice:captureDevice error:&err];
                
                // Know your errors
                if (err)
                {
                    NSLog(@"Error getting capture device input: %@", err);
                }

                // Find and remove old camera from input
                for (AVCaptureInput *oldInput in [self.captureSession inputs])
                {
                    [self.captureSession removeInput:oldInput];
                }

                // Add new camera to the input of the capture session (slang for 'switch camera')
                [self.captureSession addInput:input];
                
                // Set new capture position
                self.capturePosition = input.device.position;

                // End configuring capture session
                [self.captureSession commitConfiguration];
                
                // Give yourself a break
                break;
            }
        }
        
        // We switched the camera
        return YES;
    }

    // Nope
    return NO;
}

#pragma mark - Torch

@synthesize torchOn = _torchOn;

- (void)setTorchOn:(BOOL)torchOn
{
    AVCaptureDevice *device = [AVCaptureDevice defaultDeviceWithMediaType:AVMediaTypeVideo];
    
    if ([device hasTorch] && [device hasFlash])
    {
        [device lockForConfiguration:nil];

        if (torchOn)
        {
            [device setTorchMode:AVCaptureTorchModeOn];
            [device setFlashMode:AVCaptureFlashModeOn];
        }
        else
        {
            [device setTorchMode:AVCaptureTorchModeOff];
            [device setFlashMode:AVCaptureFlashModeOff];
        }

        [device unlockForConfiguration];
    }
}

- (BOOL)torchOn
{
    AVCaptureDevice *device = [AVCaptureDevice defaultDeviceWithMediaType:AVMediaTypeVideo];

    if ([device hasTorch] && [device hasFlash])
    {
        return device.isTorchActive;
    }

    return NO;
}

#pragma mark - AVMetadataObjectsDelegate

- (void)captureOutput:(AVCaptureOutput *)captureOutput
didOutputMetadataObjects:(NSArray *)metadataObjects
       fromConnection:(AVCaptureConnection *)connection
{
    for (AVMetadataMachineReadableCodeObject *avmmrco in metadataObjects)
    {
        id callback = ^()
        {
            if (self.didRecognizeMetadataObject)
            {
                self.didRecognizeMetadataObject (self,
                                                 captureOutput,
                                                 connection,
                                                 _previewLayer,
                                                 avmmrco);
            }
        };

        if (!self.callbackQueue)
        {
            dispatch_async(dispatch_get_main_queue(), callback);
        }
        else
        {
            dispatch_async(self.callbackQueue, callback);
        }
    }
}

@end
