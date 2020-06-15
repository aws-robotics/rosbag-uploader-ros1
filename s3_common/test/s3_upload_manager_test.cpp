/*
 * Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */
#include <condition_variable>
#include <cstdio>
#include <iostream>
#include <string>
#include <boost/bind.hpp>

#include <aws/s3/S3Client.h>
#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <s3_common/s3_common_error_codes.h>
#include <s3_common/s3_facade.h>
#include <s3_common/s3_upload_manager.h>

using namespace Aws::S3;
using ::testing::DoAll;
using ::testing::InvokeWithoutArgs;
using ::testing::IgnoreResult;
using ::testing::Return;
using ::testing::_;

class MockS3Facade : public S3Facade
{
public:
  MockS3Facade(const bool enable_encryption) : S3Facade(enable_encryption) {}
  MOCK_METHOD3(PutObject, Model::PutObjectOutcome(const std::string &, const std::string &, const std::string &));
};

class S3UploadManagerTest : public ::testing::Test
{
protected:
  bool enable_encryption;
  std::unique_ptr<MockS3Facade> facade;
  std::vector<UploadDescription> uploads;
  std::vector<UploadDescription> completed_uploads;
  std::size_t num_feedback_calls;
  Model::PutObjectOutcome successful_outcome;
  Model::PutObjectOutcome failed_outcome;

  S3UploadManagerTest() : enable_encryption(false), successful_outcome(Model::PutObjectResult()) {}

  void SetUp() override
  {
    num_feedback_calls = 0;
    facade = std::make_unique<MockS3Facade>(enable_encryption);
    uploads = 
      {
        {"file1", "location1"},
        {"file2", "location2"}
      };
  }

public:
  void FeedbackCallback(const std::vector<UploadDescription>& callback_uploads)
  {
    num_feedback_calls++;
    EXPECT_EQ(callback_uploads.size(), num_feedback_calls);
    completed_uploads = callback_uploads;
  }

  std::future<Model::PutObjectOutcome> UploadFilesUntilUnlocked(std::unique_ptr<S3UploadManager> & manager,
                            std::mutex & lock_during_uploading,
                            std::condition_variable & notify_is_uploading,
                            bool & is_uploading,
                            const boost::function<void(const std::vector<UploadDescription>&)>& callback,
                            int additional_returns = 0) {
    auto & mock_calls = EXPECT_CALL(*facade, PutObject(_,_,_))
      .WillOnce(DoAll(
        InvokeWithoutArgs([&is_uploading, &notify_is_uploading, &lock_during_uploading]() {
          is_uploading = true;
          // Notify that the function is entered and blocking
          notify_is_uploading.notify_all();
          // Block until the mutex has been unlocked
          std::unique_lock<std::mutex> lock(lock_during_uploading);
        }),
        Return(successful_outcome)));
    for (int i = 0; i < additional_returns; ++i) {
      mock_calls.WillOnce(Return(successful_outcome));
    }

    manager = std::unique_ptr<S3UploadManager>(new S3UploadManager(std::move(facade)));

    auto upload = [this, &manager, callback]() {
      return manager->UploadFiles(this->uploads, "bucket", callback);
    };

    return std::async(std::launch::async, upload);
  } 
};

TEST_F(S3UploadManagerTest, TestClientConfigConstructor)
{
  Aws::Client::ClientConfiguration config;
  S3UploadManager manager(enable_encryption, config);
  EXPECT_TRUE(manager.IsAvailable());
  auto outcome = manager.UploadFiles(uploads, "bucket",
          [this](const std::vector<UploadDescription>& callback_uploads)
          {this->FeedbackCallback(callback_uploads);});
  // This test isn't using mocks so it could fail because of invalid credentials,
  // not being able to connect to s3 or because the files that are being uploaded don't
  // exist. The purpose of this test isn't to check the error modes.
  EXPECT_FALSE(outcome.IsSuccess());
}

TEST_F(S3UploadManagerTest, TestUploadFilesSuccess)
{
  EXPECT_CALL(*facade,PutObject(_,_,_))
    .Times(2)
    .WillRepeatedly(Return(successful_outcome));

  S3UploadManager manager(std::move(facade));
  EXPECT_TRUE(manager.IsAvailable());
  EXPECT_TRUE(completed_uploads.empty());
  auto outcome = manager.UploadFiles(uploads, "bucket",
          [this](const std::vector<UploadDescription>& callback_uploads)
          {this->FeedbackCallback(callback_uploads);});
  EXPECT_TRUE(outcome.IsSuccess());
  EXPECT_EQ(uploads.size(), num_feedback_calls);
  EXPECT_EQ(uploads, completed_uploads);
  EXPECT_TRUE(manager.IsAvailable());
}

TEST_F(S3UploadManagerTest, TestUploadFilesFailsPutObjectFails)
{
  // First call succeeds, indicated by having a result.
  // Second call fails with an arbitrary error type.
  EXPECT_CALL(*facade,PutObject(_,_,_))
    .WillOnce(Return(successful_outcome))
    .WillOnce(Return(failed_outcome));
  S3UploadManager manager(std::move(facade));
  EXPECT_TRUE(manager.IsAvailable());
  auto outcome = manager.UploadFiles(uploads, "bucket",
          [this](const std::vector<UploadDescription>& callback_uploads)
          {this->FeedbackCallback(callback_uploads);});
  EXPECT_FALSE(outcome.IsSuccess());
  EXPECT_EQ(1u, num_feedback_calls);
  EXPECT_EQ(1u, completed_uploads.size());
  EXPECT_EQ(uploads.at(0), completed_uploads.at(0));
  EXPECT_TRUE(manager.IsAvailable());
}

TEST_F(S3UploadManagerTest, TestUploadFilesFailsWhileManagerUploading)
{
  std::unique_ptr<S3UploadManager> manager;
  bool is_uploading = false;
  // Pause the execution of the facade to simulate waiting for upload to S3
  std::mutex pause_mutex;
  // Used to notify main thread that the upload has started
  std::condition_variable upload_cv;

  auto feedback_callback = [this](const std::vector<UploadDescription> & callback_uploads) {
    this->FeedbackCallback(callback_uploads);
  };

  // Pause execution of file upload
  pause_mutex.lock();

  std::future<Model::PutObjectOutcome> outcome1 = UploadFilesUntilUnlocked(manager, pause_mutex, upload_cv, is_uploading, feedback_callback, 1);

  // Wait until upload has started so that manager should be busy.
  {
    std::mutex cv_mutex;
    std::unique_lock<std::mutex> lk(cv_mutex);
    upload_cv.wait(lk, [&is_uploading]() { return is_uploading; });
  }

  EXPECT_FALSE(manager->IsAvailable());

  Model::PutObjectOutcome outcome2 = manager->UploadFiles(uploads, "bucket", feedback_callback);

  // The manager is busy and should reject the upload request
  EXPECT_EQ(S3Errors::INVALID_ACTION, outcome2.GetError().GetErrorType());
  // No files have been uploaded
  EXPECT_TRUE(completed_uploads.empty());

  // Finish execution of file upload
  pause_mutex.unlock();

  // The first request should continue uninterrupted
  EXPECT_TRUE(outcome1.get().IsSuccess());
  EXPECT_EQ(uploads, completed_uploads);
  EXPECT_EQ(uploads.size(), num_feedback_calls);

  EXPECT_TRUE(manager->IsAvailable());
}

TEST_F(S3UploadManagerTest, TestCancelUpload)
{
  std::unique_ptr<S3UploadManager> manager;
  bool is_uploading = false;
  // Pause the execution of the facade to simulate waiting for upload to S3
  std::mutex pause_mutex;
  // Used to notify main thread that the upload has started
  std::condition_variable upload_cv;

  auto feedback_callback = [this](const std::vector<UploadDescription>& callback_uploads) {
    this->FeedbackCallback(callback_uploads);
  };

  // Pause execution of file upload
  pause_mutex.lock();

  std::future<Model::PutObjectOutcome> outcome = UploadFilesUntilUnlocked(manager, pause_mutex, upload_cv, is_uploading, feedback_callback);

  // Wait until upload has started so that manager should be busy.
  {
    std::mutex cv_mutex;
    std::unique_lock<std::mutex> lk(cv_mutex);
    upload_cv.wait(lk, [&is_uploading]() { return is_uploading; });
  }

  EXPECT_FALSE(manager->IsAvailable());
  // First call to cancel should succeed
  manager->CancelUpload();

  // Finish execution of file upload
  pause_mutex.unlock();

  // Canceled uploads are marked as successfull
  EXPECT_TRUE(outcome.get().IsSuccess());
  EXPECT_EQ(1u, num_feedback_calls);
  EXPECT_EQ(1u, completed_uploads.size());
  EXPECT_EQ(uploads.at(0), completed_uploads.at(0));

  EXPECT_TRUE(manager->IsAvailable());
}
