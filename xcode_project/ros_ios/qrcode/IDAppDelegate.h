//
//  IDAppDelegate.h
//  Barcodes_iOS7_Sample
//
//  Created by Ivan Djeferov on 10/12/13.
//  Copyright (c) 2013 Incept Development. All rights reserved.
//

#import <UIKit/UIKit.h>

@class IDVideoViewController;

@interface IDAppDelegate : UIResponder <UIApplicationDelegate>

@property (strong, nonatomic) UIWindow *window;
@property (strong, nonatomic) IDVideoViewController *videoViewController;

@end
