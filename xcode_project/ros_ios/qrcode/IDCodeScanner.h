//
//  IDCodeScanner.h
//  SquareCam 
//
//  Created by Ivan Djeferov on 10/5/13.
//  Copyright (c) 2013 Incept Development. All rights reserved.
//


#import <Foundation/Foundation.h>

@import AVFoundation;

#if __IPHONE_OS_VERSION_MIN_REQUIRED < __IPHONE_7_0
#warning This code is intended for iOS versions 7.0 and above
#endif

#define kIDCodeScannerParametersSessionPresetKey @"sessionPreset"

@interface IDCodeScanner : NSObject

#pragma mark - Initialize a capture session

/**
 * @brief Starts a capture session for a given camera position (front or back)
 * @param capturePosition specifies the position (front or back)
 * @param parameters Optional parameters, for now we have kIDCodeScannerParametersSessionPresetKey which is a setting for the capture session quality
 * @param metadataObjectTypes The types of meta data to recognize (Codes and Faces)
 * This method calls back on the callbackQueue with shouldAddLayerFromCaptureSessionToView if everything went well or didFailWithError if something went wrong.
 */
- (void)startCaptureSessionForCapturePosition:(AVCaptureDevicePosition)capturePosition parameters:(NSDictionary *)parameters andMetadataObjectTypes:(NSArray *)metadataObjectTypes;

/**
 * @brief The above method without optional parameters
 */
- (void)startCaptureSessionForCapturePosition:(AVCaptureDevicePosition)capturePosition andMetadataObjectTypes:(NSArray *)metadataObjectTypes;

/**
 * @brief Stops a capture session if it is running
 */
- (void)stopCaptureSession;

#pragma mark - Fancy features

/**
 * @brief Turn torch (a.k.a. flashlight) on or off
 */
@property (nonatomic, assign) BOOL torchOn;

/**
 * @brief Swap between front and back camera
 */
- (BOOL)switchCamera;

#pragma mark - Status

/**
 * @brief Current metadata objects recognized
 */
@property (nonatomic, readonly) NSArray *currentlyRecognizedMetadataObjects;

/**
 * @brief AVCaptureDevicePosition of the current capturing camera
 */
@property (nonatomic, assign) AVCaptureDevicePosition capturePosition;

/**
 * @brief Capture sessions
 * The of this dictionary key is a AVCaptureDevicePosition type and the value a capture session
 */
@property (nonatomic, readonly) AVCaptureSession *captureSession;

/**
 * @brief The preview layers containing camera output
 */
@property (nonatomic, readonly) AVCaptureVideoPreviewLayer *previewLayer;

#pragma mark - Callbacks

/**
 * @brief Queue to callback upon.
 */
@property dispatch_queue_t callbackQueue;

/**
 * @brief Callback when the layer should be added to a view
 */
@property (nonatomic, copy) void (^shouldAddLayerFromCaptureSessionToView) (IDCodeScanner *codeScanner, AVCaptureDevicePosition capturePosition, AVCaptureSession *session, AVCaptureVideoPreviewLayer *previewLayer);

/**
 * @brief Error callback
 */
@property (nonatomic, copy) void (^didFailWithError) (IDCodeScanner *codeScanner, NSError *error);

/**
 * @brief Callback when a code or face is recognized.
 */
@property (nonatomic, copy) void (^didRecognizeMetadataObject) (IDCodeScanner *codeScanner, AVCaptureOutput *output, AVCaptureConnection *connection, AVCaptureVideoPreviewLayer *previewLayer, AVMetadataMachineReadableCodeObject *code);

@end
