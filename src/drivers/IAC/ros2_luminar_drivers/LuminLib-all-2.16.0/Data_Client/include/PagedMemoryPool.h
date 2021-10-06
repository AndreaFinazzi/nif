/*
* PagedMemoryPool.h
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

#pragma once

#include <memory>
#include <mutex>

namespace lum
{
template <typename T>
struct PagedMemoryPool
{
    struct PoolItem
    {
    private:
        using StorageType = char[sizeof( T )];
        // Storage of proper size for T
        StorageType mDatum;

        // Points to the next freely available item.
        PoolItem *mpNext;

    public:
        // Methods for the list of free items.
        PoolItem *GetNextItem() const { return mpNext; }
        void SetNextItem( PoolItem *n ) { mpNext = n; }

        // Methods for the storage of the item.
        T *GetStorage() { return reinterpret_cast<T *>( &mDatum ); }

        // Given a T* cast it to a PoolItem*
        static PoolItem *StorageToItem( T *t )
        {
            PoolItem *currentItem = reinterpret_cast<PoolItem *>( t );
            return currentItem;
        }
    };  // PoolItem

    // Page of T. This is just an array of items and a pointer
    // to another page. All pages are singly linked between them.
    struct PoolPage
    {
    private:
        // Storage of this page.
        std::unique_ptr<PoolItem[]> mpStorage;

        // Pointer to the next page.
        std::unique_ptr<PoolPage> mpNext;

    public:
        // Creates a pageSize array of PoolItems.
        PoolPage( size_t pageSize )
            : mpStorage( nullptr )
            , mpNext( nullptr )
        {
            mpStorage = std::make_unique<PoolItem[]>( pageSize );
            assert( pageSize > 0 );
            for ( size_t i = 1; i < pageSize; ++i )
            {
                mpStorage[i - 1].SetNextItem( &mpStorage[i] );
            }
            mpStorage[pageSize - 1].SetNextItem( nullptr );
        }
        // Returns a pointer to the array of items. This is used by the page
        // itself. This is only used to update the pool's list of free items during initialization
        // or when creating a new page when the current one is full.
        PoolItem *GetStorage() const { return mpStorage.get(); }

        // Sets the next page. Used when the current page is full and
        // we have created this one to get more storage.
        void SetNextPage( std::unique_ptr<PoolPage> &n )
        {
            assert( !mpNext );
            mpNext.reset( n.release() );
        }
    };  // PoolPage

    // Size of the pages created by the pool.
    size_t mPageSize;

    // The maximum number of pages to be allocated by this pool, 0 for unlimited
    size_t mMaxPageCount;

    // Current number of pages allocated by this memory pool
    size_t mPageCount;

    // Current Page. Changes when it becomes full and we want to allocate one
    // more object.
    std::unique_ptr<PoolPage> mpCurrentPage;

    // List of free elements. The list can be threaded between different pages
    // depending on the deallocation pattern.
    PoolItem *mpFreeItemList;

    // Mutex for thread safe access to list of free items
    std::mutex mFreeMutex;

    // Creates a new pool that will create up to maxPages of pageSize.
    PagedMemoryPool( size_t pageSize, size_t maxPages = 0 )
        : mPageSize( pageSize )
        , mMaxPageCount( maxPages )
        , mPageCount( 1 )
        , mpCurrentPage( nullptr )
        , mpFreeItemList( nullptr )
    {
        mpCurrentPage = std::make_unique<PoolPage>( mPageSize );
        mpFreeItemList = mpCurrentPage->GetStorage();
    }

    // Allocates an object in the current page.
    template <typename... Args>
    T *alloc( Args &&... args )
    {
        std::unique_lock<std::mutex> lock( mFreeMutex );
        // If the current page is full, create a new one, unless we've reached our max page count
        // If we've reached our max, return a null pointer, the pool is full
        if ( mpFreeItemList == nullptr )
        {
            if ( mMaxPageCount == 0 || mPageCount < mMaxPageCount )
            {
                ++mPageCount;

                std::unique_ptr<PoolPage> newPage = std::make_unique<PoolPage>( mPageSize );
                // Link the new page to the current one.
                newPage->SetNextPage( mpCurrentPage );
                // Make the new page the current one.
                mpCurrentPage.reset( newPage.release() );
                // Update the free_list with the storage of the just created page.
                mpFreeItemList = mpCurrentPage->GetStorage();
            }
            else
            {
                return nullptr;
            }
        }

        // Get the first free item.
        PoolItem *currentItem = mpFreeItemList;
        // Update the free list to the next free item.
        mpFreeItemList = currentItem->GetNextItem();

        // Get the storage for T.
        T *result = currentItem->GetStorage();
        // Construct the object in the obtained storage.
        new ( result ) T( std::forward<Args>( args )... );

        return result;
    }

    void free( T *t )
    {
        // Destroy the object.
        t->T::~T();

        // Convert this pointer to T to its enclosing pointer of a PoolItem in the
        // page.
        PoolItem *currentItem = PoolItem::StorageToItem( t );
        std::unique_lock<std::mutex> lock( mFreeMutex );
        // Add the item at the beginning of the free list.
        currentItem->SetNextItem( mpFreeItemList );

        mpFreeItemList = currentItem;
    }
};  // PagedMemoryPool<T>
};  // namespace lum